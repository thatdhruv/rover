*** Settings ***
Suite Setup       Setup
Suite Teardown    Teardown
Test Teardown     Test Teardown
Resource          ${RENODEKEYWORDS}
Library           OperatingSystem

*** Variables ***
${BUILD}          ${CURDIR}/../../build/good
${BAD_UPDATE}     ${CURDIR}/../../build/bad_update.hex

*** Keywords ***
Prepare Flash And Platform
    ${erased}=    Evaluate    bytes([255]) * 0x200000
    Create Binary File    ${TEMPDIR}/erased_flash.bin    ${erased}
    Execute Command    $erased=@${TEMPDIR}/erased_flash.bin
    Execute Command    $platform=@${CURDIR}/rover_platform.repl

Boot Rover
    Prepare Flash And Platform
    Execute Command    $mcuboot=@${BUILD}/mcuboot/zephyr/zephyr.hex
    Execute Command    $app=@${BUILD}/app/zephyr/zephyr.signed.confirmed.hex
    Execute Command    include @${CURDIR}/rover.resc
    Create Terminal Tester    sysbus.usart3    timeout=30

*** Test Cases ***
Signed Image Boots Through MCUboot
    [Documentation]    MCUboot validates the ECDSA signature, jumps to slot 0, and the app starts.
    Boot Rover
    Start Emulation
    Wait For Line On Uart    rover firmware starting

Application Reaches Steady State
    [Documentation]    All startup steps (control loop, IMU task, watchdog) complete.
    Boot Rover
    Start Emulation
    Wait For Line On Uart    rover firmware running
    Wait For Line On Uart    image confirmed    timeout=30

Bad Update Rolls Back Automatically
    [Documentation]    A hanging update is staged in slot 1. MCUboot swaps it in and boots it; it hangs
    ...                before it can be confirmed, the watchdog resets the chip, and MCUboot must swap
    ...                the previous (good) image back. The good image then runs and confirms itself,
    ...                which the bad image can never do because it hangs after one second.
    Prepare Flash And Platform
    Execute Command    $mcuboot=@${BUILD}/mcuboot/zephyr/zephyr.hex
    Execute Command    $app=@${BUILD}/app/zephyr/zephyr.signed.confirmed.hex
    Execute Command    include @${CURDIR}/rover.resc
    Execute Command    sysbus LoadHEX @${BAD_UPDATE}
    Create Terminal Tester    sysbus.usart3    timeout=60
    Start Emulation
    Wait For Line On Uart    TEST IMAGE
    Wait For Line On Uart    rover firmware starting
    Wait For Line On Uart    rover firmware running
    Wait For Line On Uart    image confirmed    timeout=60
