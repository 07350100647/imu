/*
 *@startuml

participant app as ap
participant bsp_imu as bsp
participant sh2_hal as sh2_hal
participant sh2 as sh2
participant shtp as shtp

== Init IMU module ==
ap --> bsp : bsp_imu_init()
activate bsp
bsp -> bsp : init I/O, SPI and EXTI
bsp -> sh2_hal : sh2_hal_init()

bsp -> ap : SDCard_SPI_Handle_t *
deactivate bsp

== tx_cycle ==
sh2 -> shtp : shtp_send(...)

activate sh2
activate shtp
shtp -> shtp : txProcess (...)

activate shtp
shtp -> sh2_hal :  sh2_hal_tx(...)

activate sh2_hal
sh2_hal -> bsp : tx_shtp(...)

activate bsp
bsp -> bsp : try_mutex
bsp --> imu : waken
return SH2_OK or SH2_ERROR
return result

alt result=SH2_OK
shtp -> shtp : continuation=true

return status

return rc

imu --> bsp : IMU_EXTI_Callback
imu -> imuNotify : EVT_INTN

activate imuNotify
imuNotify -> imuNotify : state = DEV_IN_PROG
imuNotify -> bsp : startOpShtp

activate bsp

bsp -> bsp : transferPhase  = TRANSFER_HDR
bsp -> spi : receive HDR
spi <-> imu : data exchange
activate imu


spi --> bsp: HAL_SPI_TxRxCpltCallback

bsp -> bsp : transferPhase = TRANSFER_DATA
bsp -> spi : receive payload
spi <-> imu : data exchange

spi --> bsp : HAL_SPI_TxRxCpltCallback

bsp -> bsp : transferPhase = TRANSFER_IDLE
bsp -> imuNotify : EV_CPLT

imuNotify -> bsp : endOpShtp

imuNotify ->] : deliverRX

@enduml
*/

/*
 @startuml

participant app as ap
participant sh2 as sh2
participant shtp as shtp
participant bsp_imu as bsp

== Init IMU module ==
ap --> bsp : bsp_imu_init(hdlr, *sensor_cb,\n\t*block, *unblock, *delay_f)
activate bsp
bsp -> bsp : init I/O, SPI, EXTI,\nreset IMU and \nstore support functions.

bsp -> sh2 : sh2_initialize(evt_cb)
note left
    This function will deassert reset.
end note

bsp -> sh2 : sh2_setSensorCallback (sensor_cb)
...
note over ap, bsp
Wait for reset event
end note
...
return bsp_imu_Handle_t *
deactivate bsp


 */
/*
@startuml

participant app as ap
participant bsp_imu as bsp
participant sh2 as sh2

== get Products IDs ==

ap -> sh2 : sh2_getProdIds(&prodIDs)
activate sh2
sh2 -> bsp :  sh2_hal_block()
bsp -> ap : block()
...
note over ap, bsp
App must be hold until unblock
is asserted from ISR
end note
...
sh2 -\ bsp : sh2_hal_unblock()
bsp -\ ap : unblock()

return status
deactivate sh2

note over ap, bsp
Data available on prodIDs, if no error.

This will result in a list like this:
**Part 10003608 : Version 3.2.7 Build 370**
**Part 10003606 : Version 1.2.4 Build 230**
**Part 10003254 : Version 4.4.2 Build 405**
**Part 10003171 : Version 4.2.7 Build 473**
end note

@enduml
*/
