#!/bin/bash

# This build script assumes that the kauailabs allwpilib repo, and 
# any other needed repos, have already been acquired.

set -x

for BUILDTYPE in Debug Release
# for BUILDTYPE in Release
do
	for BUILD_CPP_ONLY in ON OFF
	# for BUILD_CPP_ONLY in OFF
	do

		if [ "$BUILD_CPP_ONLY" == "ON" ]; then
			JAVA_ENABLED=OFF
			BUILD_DIR=${BUILDTYPE}_Static
		else
			JAVA_ENABLED=ON
			BUILD_DIR=${BUILDTYPE}_Shared
		fi

		cd ~
		sudo rm -r wpilib_build_${BUILD_DIR}
		mkdir wpilib_build_${BUILD_DIR}/
		cd wpilib_build_${BUILD_DIR}
		# export JAVA_HOME=//wsl.localhost/Ubuntu/usr/lib/jvm/java-11-openjdk-amd64
		export JAVA_HOME=/usr/lib/jvm/java-11-openjdk-armhf
		cmake -D WITH_JAVA=${JAVA_ENABLED} -D WITH_TESTS=OFF -D CMAKE_BUILD_TYPE=${BUILDTYPE} -D BUILD_SHARED_LIBS=${JAVA_ENABLED} -D WITH_SIMULATION_MODULES=${JAVA_ENABLED} -D OpenCV_DIR=/home/pi/opencv/build /home/pi/allwpilib
		# cmake -D WITH_JAVA=OFF -D BUILD_SHARED_LIBS=OFF -D WITH_SIMULATION_MODULES=OFF -D WITH_TESTS=OFF -D WITH_NTCORE=ON -D WITH_WPIMATH=ON -D WITH_GUI=OFF -D WITH_CSCORE=OFF -D WITH_WPILIB=ON -DCMAKE_BUILD_TYPE=${BUILDTYPE} -DBUILD_SHARED_LIBS=${SHARED_LIBS} -D OpenCV_DIR=/home/pi/opencv/build /home/pi/allwpilib
		# cmake -D WITH_ONLY_HAL=ON -D WITH_JAVA=OFF -D BUILD_SHARED_LIBS=OFF -D WITH_SIMULATION_MODULES=OFF -D WITH_TESTS=OFF -D WITH_NTCORE=OFF -D WITH_WPIMATH=OFF -D WITH_GUI=OFF -D WITH_CSCORE=OFF -D WITH_WPILIB=OFF -DCMAKE_BUILD_TYPE=${BUILDTYPE} -DBUILD_SHARED_LIBS=${SHARED_LIBS} -D OpenCV_DIR=/home/pi/opencv/build /home/pi/allwpilib		
		# cmake -D WITHOUT_ALLWPILIB=OFF -D WITHOUT_JAVA=${BUILD_CPP_ONLY} -DCMAKE_BUILD_TYPE=${BUILDTYPE} -DBUILD_SHARED_LIBS=${SHARED_LIBS} -D OpenCV_DIR=/home/pi/opencv-3.4.7/build /home/pi/allwpilib
		make
		sudo make install
		cd hal
		gradle --rerun-tasks publishMavenPublicationToMavenLocal

		# Copy VMX-pi Channel Maps.  Note that this copying is now optional; defaults are used if the ChannelMap is not present.
		sudo cp -r /home/pi/allwpilib/hal/src/main/native/raspi/Translator/Maps/ChannelMap.json /usr/local/frc/third-party/lib

		# Not sure why this is here - it may no longer be necessary
		sudo mkdir /tmp/frc_versions
	done
done

# Once all the builds are complete, publish the final results to MavenCentral
cd ~
cd ~/.m2/repository/com/kauailabs/vmx/first/hal/hal-cpp
latest_subdir=$(ls -td -- */ | head -n 1)
cd $latest_subdir
ls *
jar -cvf bundle.jar *.pom *.asc *.zip
ls *

publish_bundle_dir=$(pwd)
echo "******************************************************"
echo
echo "Upload the bundle file ($publish_bundle_dir/bundle.jar) as an Artifact Bundle on the Staging Upload page at the Sonatype OSS site (oss.sonatype.org)"
echo
echo "******************************************************"
