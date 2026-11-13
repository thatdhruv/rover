#include <string.h>

#include <zephyr/storage/flash_map.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/crc.h>

#include <rover/faultlog.h>

#define REC_SIZE  32
#define REC_MAGIC 0xFA17

/* on-flash layout, little endian */
struct rec_raw {
	uint16_t magic;
	uint16_t code;
	uint32_t seq;
	uint32_t uptime_ms;
	uint32_t arg0;
	uint32_t arg1;
	uint8_t pad[8];
	uint32_t crc;
} __packed;

BUILD_ASSERT(sizeof(struct rec_raw) == REC_SIZE);

static bool slot_blank(const struct rec_raw *r)
{
	const uint8_t *p = (const uint8_t *)r;

	for (size_t i = 0; i < REC_SIZE; i++) {
		if (p[i] != 0xFF) {
			return false;
		}
	}
	return true;
}

static bool slot_valid(const struct rec_raw *r)
{
	return sys_le16_to_cpu(r->magic) == REC_MAGIC &&
	       sys_le32_to_cpu(r->crc) == crc32_ieee((const uint8_t *)r, REC_SIZE - 4);
}

static int slot_read(struct rover_faultlog *l, uint32_t slot, struct rec_raw *r)
{
	return flash_area_read(l->fa, slot * REC_SIZE, r, REC_SIZE);
}

int rover_faultlog_init(struct rover_faultlog *l, uint8_t flash_area_id)
{
	struct flash_sector sector;
	uint32_t sectors = 1;
	struct rec_raw r;
	uint32_t last_slot = 0;
	uint32_t last_seq = 0;
	bool any = false;
	int ret;

	memset(l, 0, sizeof(*l));
	ret = flash_area_open(flash_area_id, &l->fa);
	if (ret < 0) {
		return ret;
	}
	ret = flash_area_get_sectors(flash_area_id, &sectors, &sector);
	if (ret < 0 && ret != -ENOMEM) {
		return ret;
	}
	/* -ENOMEM means more sectors than our single-entry buffer; sector 0 is valid.
	 * The partition must be made of uniform erase sectors.
	 */
	l->sector_size = sector.fs_size;
	l->slots_per_sector = l->sector_size / REC_SIZE;
	l->slots = l->fa->fa_size / l->sector_size * l->slots_per_sector;
	if (l->slots_per_sector == 0 || l->slots == 0) {
		return -EINVAL;
	}

	for (uint32_t s = 0; s < l->slots; s++) {
		ret = slot_read(l, s, &r);
		if (ret < 0) {
			return ret;
		}
		if (slot_valid(&r) && (!any || (int32_t)(sys_le32_to_cpu(r.seq) - last_seq) > 0)) {
			last_seq = sys_le32_to_cpu(r.seq);
			last_slot = s;
			any = true;
		}
	}

	l->next_seq = any ? last_seq + 1 : 1;
	l->next_slot = any ? (last_slot + 1) % l->slots : 0;

	/* If the head slot is dirty (torn write), skip to the next blank one. */
	for (uint32_t tries = 0; tries < l->slots; tries++) {
		if (l->next_slot % l->slots_per_sector == 0) {
			break; /* sector will be erased when we write into it */
		}
		ret = slot_read(l, l->next_slot, &r);
		if (ret < 0) {
			return ret;
		}
		if (slot_blank(&r)) {
			break;
		}
		l->next_slot = (l->next_slot + 1) % l->slots;
	}
	return 0;
}

int rover_faultlog_append(struct rover_faultlog *l, uint16_t code, uint32_t arg0, uint32_t arg1,
			  uint32_t uptime_ms)
{
	struct rec_raw r = {0};
	int ret;

	if (l->next_slot % l->slots_per_sector == 0) {
		ret = flash_area_erase(l->fa, (l->next_slot / l->slots_per_sector) * l->sector_size,
				       l->sector_size);
		if (ret < 0) {
			return ret;
		}
	}

	r.magic = sys_cpu_to_le16(REC_MAGIC);
	r.code = sys_cpu_to_le16(code);
	r.seq = sys_cpu_to_le32(l->next_seq);
	r.uptime_ms = sys_cpu_to_le32(uptime_ms);
	r.arg0 = sys_cpu_to_le32(arg0);
	r.arg1 = sys_cpu_to_le32(arg1);
	memset(r.pad, 0, sizeof(r.pad));
	r.crc = sys_cpu_to_le32(crc32_ieee((const uint8_t *)&r, REC_SIZE - 4));

	ret = flash_area_write(l->fa, l->next_slot * REC_SIZE, &r, REC_SIZE);
	if (ret < 0) {
		return ret;
	}
	l->next_seq++;
	l->next_slot = (l->next_slot + 1) % l->slots;
	return 0;
}

uint32_t rover_faultlog_count(struct rover_faultlog *l)
{
	struct rec_raw r;
	uint32_t n = 0;

	for (uint32_t s = 0; s < l->slots; s++) {
		if (slot_read(l, s, &r) == 0 && slot_valid(&r)) {
			n++;
		}
	}
	return n;
}

int rover_faultlog_read(struct rover_faultlog *l, uint32_t age, struct rover_fault_rec *out)
{
	const uint32_t want = l->next_seq - 1 - age;
	struct rec_raw r;

	if (age >= l->next_seq - 1) {
		return -ENOENT;
	}
	for (uint32_t s = 0; s < l->slots; s++) {
		if (slot_read(l, s, &r) == 0 && slot_valid(&r) && sys_le32_to_cpu(r.seq) == want) {
			out->seq = want;
			out->uptime_ms = sys_le32_to_cpu(r.uptime_ms);
			out->code = sys_le16_to_cpu(r.code);
			out->arg0 = sys_le32_to_cpu(r.arg0);
			out->arg1 = sys_le32_to_cpu(r.arg1);
			return 0;
		}
	}
	return -ENOENT;
}

int rover_faultlog_clear(struct rover_faultlog *l)
{
	int ret = flash_area_erase(l->fa, 0, l->fa->fa_size);

	if (ret == 0) {
		l->next_slot = 0;
		/* sequence numbers keep increasing so ordering stays monotonic */
	}
	return ret;
}
