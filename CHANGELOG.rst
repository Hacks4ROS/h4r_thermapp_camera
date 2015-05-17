^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package h4r_thermapp_camera
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.1 (2015-05-17)
------------------
* fix License information
* add install to CMakeLists
* update ReadMe with install information
* add udev file
* change thermapp.launch
* Merge branch 'develop'
* fix submodule for ThermAppCam project
* Initial commit
* add author tag
* fix dynamic reconfigure
* change #define for header file
* change package name to h4r
  add default values to dynamic_reconfigure
  tidy up package.xml
* integrate reading camera
* rename namespace,
  add first functions
  make it compile with ThermAppCam files
* delete old thermapp code
* delete class for own code
* add new thermapp code to library and to node in CMakeLists
* create include for catkin for not breaking up with origin of the thermapp code
* update ThermAppCam
* add ThermAppCam from PidBip (Alexander G)
* last of my own tries to get protocol working
* removed prints
* stable image but slow
* fix filename of perspective in launchfile
* reformat launchfile code
* add current code to main node to have easier testing
* add current code to bulk class
* create rqt perspective with image view and launchfile
* add Code from test application
* add function for opening usb device
  add function for closing
  add function for requesting image (unfinished)
* add libusb to CMakeLists
* add class for usb bulk transfer
* add thermapp_camera_node
  -add boost thread bind
  -add dynamic reconfigure
  -add CameraPublisher
  -add cv bridge
* Add maintainer and description to package.xml
* initial commit, create ros package
* initial commit, create ros package
* Contributors: Christian Holl
