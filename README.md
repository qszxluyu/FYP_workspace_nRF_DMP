# FYP_workspace
Workspace for Yu

This is the workspace for personal project. MPU9250 and nrf52840 is used.

The project folder is located in ...\nRF5_SDK_13.0.0_04a0bfd\examples\peripheral

Update May 27.2018: Twi busy error is now partially fixed. The reason Twi busy error happens is that calling NRF_LOG_FLUSH() when deferred log buffer is too long. To fix that just call it every time something is added to the buffer. Tested at the 10Hz output rate.

contact me: yl18914@ic.ac.uk
