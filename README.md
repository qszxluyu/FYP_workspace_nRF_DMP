# FYP_workspace
Workspace for Yu

This is the workspace for personal project. MPU9250 and nrf52840 is used.

The project folder is located in ...\nRF5_SDK_13.0.0_04a0bfd\examples\peripheral

Update May 27.2018: 
  1.Twi busy error is now partially fixed. One of the reasons that Twi busy error happens is that calling NRF_LOG_FLUSH() when deferred log buffer is too long. To fix that just call it every time something is added to the buffer. Tested at the 10Hz output rate.

Update May 29.2018: 
  1.Twi busy error is now fixed. The major reason is the incorrect compass orientation setting. After replacing with the correct compass    orientation setting, DMP and MPL works at all output rate settings(10hz, 40hz, 50hz, and 100hz).
  2.Invensense DMP and MPL supported! The porting has done, but the code haven't been modified for the offical eMPL-client.py file. Check documentation of motion_driver_6.12.
  3.Use printf instead of logger module to print out the data. Deffered logger will cause some unpredictable bugs so it is disabled(I have no idea why I've chosen to use it at the beginning). 
  4.Modified the output format for use with the FYP_Unity3D_Project. The uart input is not supported yet so have to reset the nrf52840DK to run MPU9250 self test.

contact me: yl18914@ic.ac.uk
