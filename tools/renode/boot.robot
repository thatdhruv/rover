*** Settings ***
Suite Setup       Setup
Suite Teardown    Teardown
Test Teardown     Test Teardown
Resource          ${RENODEKEYWORDS}

*** Variables ***
${MERGED}         ${CURDIR}/../../build/merged.hex

*** Keywords ***
Boot Rover
    Execute Command    $merged=@${MERGED}
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
