*** Settings ***
Suite Setup       Setup
Suite Teardown    Teardown
Test Teardown     Test Teardown
Resource          ${RENODEKEYWORDS}

*** Variables ***
${BUILD}          ${CURDIR}/../../build/good
${BAD_UPDATE}     ${CURDIR}/../../build/bad_update.hex

*** Keywords ***
Boot Rover
    Execute Command    $platform=@${CURDIR}/rover_platform.repl
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
    [Documentation]    A hung update is staged in slot 1. MCUboot swaps it in and boots it; it hangs
    ...                before being confirmed, the watchdog resets the chip, and MCUboot must swap
    ...                the previous (good) image back, which then runs and confirms itself.
    Execute Command    $platform=@${CURDIR}/rover_platform.repl
    Execute Command    $mcuboot=@${BUILD}/mcuboot/zephyr/zephyr.hex
    Execute Command    $app=@${BUILD}/app/zephyr/zephyr.signed.confirmed.hex
    Execute Command    include @${CURDIR}/rover.resc
    Execute Command    sysbus LoadHEX @${BAD_UPDATE}
    Create Terminal Tester    sysbus.usart3    timeout=60
    Start Emulation
    Wait For Line On Uart    test: simulating a hung image
    Wait For Line On Uart    rover firmware starting
    Wait For Line On Uart    rover firmware running
    Wait For Line On Uart    image confirmed    timeout=60
