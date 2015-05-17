# h4r_thermapp_camera
The thermapp_camera package provides a library and a node for the Opgal® ThermApp® Android Thermal Camera

Copy the file udev/99-opgal.rules to /etc/udev/rules.d to allow users access to the camera. 

For it to work, udev has to be restarted by

	service udev restart

and the camera must be removed from USB and attached again, if it was already plugged.
