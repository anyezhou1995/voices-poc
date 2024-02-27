#!/bin/bash

source ~/.voices_site_config
source ~/.voices_scenario_config

username=$(whoami)

function print_help {
	echo
	echo "usage: buildTenaAdapters.sh [--no_branch_change] [--no_pull] [--help]"
	echo
	echo "Start CARLA Simulator for VOICES"
	echo
	echo "optional arguments:"
	echo "    --no_branch_change	do not change repository branches"
	echo "    --no_pull          	do not pull the latest code from the repository"
	echo "    --release          	build the release version of the source code"
	echo "								NOTE: can not be used with --debug"
	echo "    --debug          		build the debug version of the source code"
	echo "								NOTE: can not be used with --release"
	echo "    --no_docker_rebuild	do not pull rebuild the build docker container"
	echo "    --help				show help"
	echo
}

arg_no_branch_change=false
arg_no_pull=false
arg_release_or_debug=false
arg_no_docker_rebuild=false
arg_github_token=''

for arg in "$@"
do
	
	if [[ $arg == "--no_branch_change" ]]; then
		
		arg_no_branch_change=true

	elif [[ $arg == "--no_pull" ]]; then
		
		arg_no_pull=true

	elif [[ $arg == "--no_docker_rebuild" ]]; then
		
		arg_no_docker_rebuild=true

	elif [[ $arg == "--release" ]]; then

		if [ ! $arg_release_or_debug == false ]; then
			echo
			echo "ERROR: --release or --debug flag used more than once"
			print_help
			exit
		fi
		
		arg_release_or_debug=1
		releaseOrDebug=1
	
	elif [[ $arg == "--debug" ]]; then
		echo arg_release_or_debug: $arg_release_or_debug

		if [ ! $arg_release_or_debug == false ]; then
			echo
			echo "ERROR: --release or --debug flag used more than once"
			print_help
			exit
		fi

		arg_release_or_debug=2
		releaseOrDebug=2

	elif [[ $arg == "--help" ]]; then
		
		print_help
		exit

	elif [[ $arg != "" ]]; then
		
		echo
		echo "Invalid argument: $arg"
		print_help
		exit

	fi
done

#-------------------| LOCAL VARIABLES |-------------------#
# localTenaDir=$VUG_LOCAL_TENA_PATH
# localTenaPackageDownloadDir=/home/$username/Downloads/TENA	#location of the TENA dependency packages
VUG_LOCAL_TENADEV_DIR=$VUG_LOCAL_TENADEV_DIR			#location of local tenadev
localInstallDir=$VUG_LOCAL_INSTALL_PATH		#location to install/build TENA adapters
localVoicesPocDir=$VUG_LOCAL_VOICES_POC_PATH
numBuildJobs=4    # number of build jobs to speed up compilation
#---------------------------------------------------------#

#-------------------| TENA VARIABLES |-------------------#
tenaVersion=6.0.8
tenaBuildVersion=u2004-gcc9-64
# VUG_TENA_SOURCE_SCRIPT_FILE=$VUG_TENA_SOURCE_SCRIPT_FILE
#--------------------------------------------------------#


#-------------------| REMOTE VARIABLES |-------------------#
#-------------------|  DO NOT CHANGE   |-------------------#
# remoteDownloadsDir=/home/Downloads 		#DO NOT CHANGE: internal docker directory mapped to localTenaPackageDownloadDir
remoteTenaDir=/home/TENA			#DO NOT CHANGE: internal docker directory mapped to localTenaDir
remoteInstallDir=/home/INSTALL		#DO NOT CHANGE: internal docker directory mapped to localInstallDir	
remoteCarlaDir=/home/carla
#--------------------------------------------------------#

middlewareVersion="MiddlewareSDK-v6.0.8"

# boostVersion="TENA_boost_1.77.0.1_Library"
# vugCombinedVersion="VUG-Combined-v1.0.0"


vugThreadsVersion="vug-threads-2.2.0"
vugUdbProtocolioVersion="vug-udp-protocolio-2.2.1"


v2xhubGitUrl="https://github.com/usdot-fhwa-OPS/V2X-Hub.git"

# if [[ -f $VUG_TENA_SOURCE_SCRIPT_FILE ]]; then
# 	source $VUG_TENA_SOURCE_SCRIPT_FILE
# else
# 	echo
# 	echo "TENA Source script not found..."
# 	exit
# fi

vug_carla_adapter_name="vug-carla-adapter"

echo
echo "What application would you like to install? [#]" 
echo 
echo "    [1]  vug-threads"
echo "    [2]  vug-udp-protocolio"
echo "    [3]  scenario-publisher"
echo "    [4]  $vug_carla_adapter_name"
echo "    [5]  tena-j2735-adapter"
echo "    [6]  tena-j3224-adapter"
echo "    [7]  tena-v2x-adapter"
echo "    [8]  tena-entity-generator"
echo "    [9]  tena-traffic-light-entity-generator"
echo "    [10] carma-platform-tena-adapter"
echo "    [11] v2xhub-tena-bsm-plugin"
echo "    [12] v2xhub-tena-spat-plugin"
echo "    [13] v2xhub-tena-mobility-plugin"
echo "    [14] v2xhub-tena-traffic-control-plugin"
echo
read -p "--> " tenaAppIndex

carlaTenaAdapterGitUrl="https://github.com/usdot-fhwa-stol/vug-carla-adapter.git"

if [[ $tenaAppIndex == 1 ]]; then
	tenaApp=vug-threads
	gitCloneUrl="https://github.com/usdot-fhwa-stol/vug-threads-library.git"
	dockerContainer=usdotfhwastoldev/voices:build-general-latest
	remoteAppDir=/home/$tenaApp	#DO NOT CHANGE: internal docker directory mapped to localAppDir
	isV2xhubPlugin=false
	requiresProtocolio=false
	useMasterDefaultBranch=false
	noBuildVersion=true

elif [[ $tenaAppIndex == 2 ]]; then
	tenaApp=vug-udp-protocolio
	gitCloneUrl="https://github.com/usdot-fhwa-stol/vug-udp-protocolio.git"
	dockerContainer=usdotfhwastoldev/voices:build-general-latest
	remoteAppDir=/home/$tenaApp	#DO NOT CHANGE: internal docker directory mapped to localAppDir
	isV2xhubPlugin=false
	requiresProtocolio=false
	useMasterDefaultBranch=false
	noBuildVersion=true

elif [[ $tenaAppIndex == 3 ]]; then
	tenaApp=vug-scenario-publisher
	gitCloneUrl="https://github.com/usdot-fhwa-stol/vug-scenario-publisher.git"
	dockerContainer=usdotfhwastoldev/voices:build-general-latest
	remoteAppDir=/home/$tenaApp	#DO NOT CHANGE: internal docker directory mapped to localAppDir
	isV2xhubPlugin=false
	requiresProtocolio=false
	useMasterDefaultBranch=false
	noBuildVersion=false

elif [[ $tenaAppIndex == 4 ]]; then
	tenaApp=$vug_carla_adapter_name
	gitCloneUrl=$carlaTenaAdapterGitUrl
	dockerContainer=usdotfhwastoldev/voices:build-carla-latest
	remoteAppDir=/home/$tenaApp 			#DO NOT CHANGE: internal docker directory mapped to localAppDir
	isV2xhubPlugin=false
	requiresProtocolio=false
	useMasterDefaultBranch=false
	noBuildVersion=false

elif [[ $tenaAppIndex == 5 ]]; then
	# tenaApp=tena-j2735-message-adapter
	tenaApp=vug-j2735-adapter
	gitCloneUrl="https://github.com/usdot-fhwa-stol/vug-j2735-adapter.git"
	dockerContainer=usdotfhwastoldev/voices:build-general-latest
	remoteAppDir=/home/$tenaApp 			#DO NOT CHANGE: internal docker directory mapped to localAppDir
	isV2xhubPlugin=false
	requiresProtocolio=false
	useMasterDefaultBranch=false
	noBuildVersion=false

elif [[ $tenaAppIndex == 6 ]]; then
	tenaApp=vug-j3224-adapter
	gitCloneUrl="https://github.com/usdot-fhwa-stol/vug-J3224-adapter.git"
	dockerContainer=usdotfhwastoldev/voices:build-general-latest
	remoteAppDir=/home/$tenaApp 			#DO NOT CHANGE: internal docker directory mapped to localAppDir
	isV2xhubPlugin=false
	requiresProtocolio=false
	useMasterDefaultBranch=false
	noBuildVersion=false

elif [[ $tenaAppIndex == 7 ]]; then
	tenaApp=vug-v2x-adapter
	gitCloneUrl="https://github.com/usdot-fhwa-stol/vug-v2x-adapter.git"
	dockerContainer=usdotfhwastoldev/voices:build-general-latest
	remoteAppDir=/home/$tenaApp 			#DO NOT CHANGE: internal docker directory mapped to localAppDir
	isV2xhubPlugin=false
	requiresProtocolio=false
	useMasterDefaultBranch=false
	noBuildVersion=false

elif [[ $tenaAppIndex == 8 ]]; then
	tenaApp=vug-entity-generator
	gitCloneUrl="https://github.com/usdot-fhwa-stol/vug-entity-generator.git"
	dockerContainer=usdotfhwastoldev/voices:build-general-latest
	remoteAppDir=/home/$tenaApp	#DO NOT CHANGE: internal docker directory mapped to localAppDir
	isV2xhubPlugin=false
	requiresProtocolio=false
	useMasterDefaultBranch=false
	noBuildVersion=false

elif [[ $tenaAppIndex == 9 ]]; then
	tenaApp=vug-traffic-light-entity-generator
	gitCloneUrl="https://github.com/usdot-fhwa-stol/vug-traffic-light-entity-generator.git"
	dockerContainer=usdotfhwastoldev/voices:build-general-latest
	remoteAppDir=/home/$tenaApp 			#DO NOT CHANGE: internal docker directory mapped to localAppDir
	isV2xhubPlugin=false
	requiresProtocolio=false
	useMasterDefaultBranch=false
	noBuildVersion=false

elif [[ $tenaAppIndex == 10 ]]; then
	tenaApp=vug-carma-platform-adapter
	gitCloneUrl="https://github.com/usdot-fhwa-stol/vug-carma-platform-adapter.git"
	dockerContainer=usdotfhwastoldev/voices:build-general-latest
	remoteAppDir=/home/$tenaApp	#DO NOT CHANGE: internal docker directory mapped to localAppDir
	isV2xhubPlugin=false
	requiresProtocolio=true
	useMasterDefaultBranch=false
	noBuildVersion=false

elif [[ $tenaAppIndex == 11 ]]; then
	tenaApp=vug-v2xhub-bsm-plugin
	gitCloneUrl="https://github.com/usdot-fhwa-stol/vug-v2xhub-bsm-plugin.git"
	dockerContainer=usdotfhwaops/v2xhubamd:voices-pilot2-latest
	remoteAppDir=/home/V2X-Hub/src/$tenaApp	#DO NOT CHANGE: internal docker directory mapped to localAppDir
	isV2xhubPlugin=true
	requiresProtocolio=false
	useMasterDefaultBranch=false
	noBuildVersion=false

elif [[ $tenaAppIndex == 12 ]]; then
	tenaApp=vug-v2xhub-spat-plugin
	gitCloneUrl="https://github.com/usdot-fhwa-stol/vug-v2xhub-spat-plugin.git"
	dockerContainer=usdotfhwaops/v2xhubamd:voices-pilot2-latest
	remoteAppDir=/home/V2X-Hub/src/$tenaApp	#DO NOT CHANGE: internal docker directory mapped to localAppDir
	isV2xhubPlugin=true
	requiresProtocolio=false
	useMasterDefaultBranch=false
	noBuildVersion=false
	
elif [[ $tenaAppIndex == 13 ]]; then
	tenaApp=vug-v2xhub-mobility-plugin
	gitCloneUrl="https://github.com/usdot-fhwa-stol/vug-v2xhub-mobility-plugin.git"
	dockerContainer=usdotfhwaops/v2xhubamd:voices-pilot2-latest
	remoteAppDir=/home/V2X-Hub/src/$tenaApp	#DO NOT CHANGE: internal docker directory mapped to localAppDir
	isV2xhubPlugin=true
	requiresProtocolio=false
	useMasterDefaultBranch=false
	noBuildVersion=false
	
elif [[ $tenaAppIndex == 14 ]]; then
	tenaApp=vug-v2xhub-traffic-control-plugin
	gitCloneUrl="https://github.com/usdot-fhwa-stol/vug-v2xhub-traffic-control-plugin.git"
	dockerContainer=usdotfhwaops/v2xhubamd:voices-pilot2-latest
	remoteAppDir=/home/V2X-Hub/src/$tenaApp	#DO NOT CHANGE: internal docker directory mapped to localAppDir
	isV2xhubPlugin=true
	requiresProtocolio=false
	useMasterDefaultBranch=false
	noBuildVersion=false

else
	echo "Invalid selection, try again..."
	exit
fi

localAppDir=$VUG_LOCAL_TENADEV_DIR/$tenaApp	#location of git directory of application to be built
			
downloadedSource=false


if [[ ! -d $localAppDir ]]; then
	echo
	read -p "Application directory not found. Would you like to download the repository? [y/n] " downloadApp
	
	if $useMasterDefaultBranch; then
		defaultBranch=master
	else
		defaultBranch=develop
	fi

	read -p "What branch would you like to use? [leave blank for $defaultBranch] " branchToDownload
	
	if [[ $downloadApp =~ ^[yY]$ ]]; then
	
		if [[ $branchToDownload == "" ]]; then
			git clone $gitCloneUrl -b $defaultBranch $localAppDir || exit
		else
			git clone $gitCloneUrl -b $branchToDownload $localAppDir || exit
		fi
	else
		echo "Please clone the latest application repository..."
	fi

	downloadedSource=true
else
	echo "$tenaApp directory found"
fi

cd $localAppDir

if ! $downloadedSource; then
	gitCommitId=$(git rev-parse HEAD)
	gitBranch=$(git rev-parse --abbrev-ref HEAD)
	gitInfo=$(git show $gitCommitId  | sed 's/^/    /' )
	gitInfo="$(echo $gitInfo | sed 's/diff .*//' )"

	echo
	echo Current Branch: $gitBranch
	

	if [ $arg_no_branch_change == false ]; then
		echo
		read -p "Would you like to switch branches? [y/n] " switchBranch

		if [[ $switchBranch =~ ^[yY]$ ]]; then
			read -p "    Enter the desired branch name --> " newBranch

			git checkout $newBranch || exit
		fi
	fi

	if [ $arg_no_pull == false ]; then 
	
		echo
		echo "Current Commit Info:"
		echo
		echo "$gitInfo"

		echo
		read -p "Would you like to pull the latest code? [y/n] " pullLatest

		if [[ $pullLatest =~ ^[yY]$ ]]; then
			echo
			git pull || exit
		fi
	fi
fi

if $noBuildVersion; then
	buildVersion=""
	buildVersionCaps=""
	buildVersionDirArg=""
	buildVersionCmakeArg=""
else

	if [ $arg_release_or_debug == false ] ; then
		echo
		echo "Would you like to build release or debug? [#]" 
		echo 
		echo "    [1]  	release"
		echo "    [2]  	debug"
		echo
		read -p "--> " releaseOrDebug
	fi

	if [[ $releaseOrDebug == 1 ]]; then
		buildVersion="release"
		buildVersionDirArg="-B release"
		buildVersionCmake="RELEASE"
		buildVersionCmakeArg="-D CMAKE_BUILD_TYPE=RELEASE"
		
	elif [[ $releaseOrDebug == 2 ]]; then
		buildVersion="-B debug"
		buildVersionDirCmd=
		buildVersionCaps="-D CMAKE_BUILD_TYPE=DEBUG"

	else
		echo
		echo "Invalid selection, try again..."
		exit
	fi
fi


##### Look for required packages to be installed

echo
echo "The looking for packages to be installed:"


## TODO: replace these check with a docker exec command into build container

# look for middleware
# if [ -d $localTenaDir/$tenaVersion ]; then
# 	echo "TENA Middleware $tenaVersion found..."
# else
# 	echo
# 	echo "The proper TENA Middleware version was not found. Please install version $tenaVersion"
# 	exit
# fi

# #look for boost
# if [ -d $localTenaDir/$boostVersion* ]; then
# 	echo "$boostVersion found..."
# else
# 	echo "The proper Boost version was not found. Please install version $boostVersion"
# 	exit
# fi

#look for VUG Combined
# if [ -d $localTenaDir/$tenaVersion/src/$vugCombinedVersion* ]; then
# 	echo "$vugCombinedVersion found..."
# else
# 	echo "The proper VUG-Combined was not found. Please install version $vugCombinedVersion"
# 	exit
# fi

#look for VUG Threads
# set -x
if [ $tenaApp == "vug-threads" ] || [ -d $localInstallDir/$vugThreadsVersion ]; then
	echo "vug-threads found..."
else
	echo "vug-threads was not found. Please install vug-threads"
	exit
fi

#look for VUG ProtocolIO
if $requiresProtocolio; then
	if [ -d $localInstallDir/$vugUdbProtocolioVersion ]; then
		echo "ProtocolIO found..."
	else
		echo "vug-udp-protocolio was not found. Please install vug-udp-protocolio"
		exit
	fi
fi

currentDockerImages=$(sudo docker image list -q $dockerContainer)

build_container_exists=false

if [[ -n $currentDockerImages ]] ; then
	echo
	echo "Found build docker container: $dockerContainer"
	build_container_exists=true
else
	echo
	echo "Build docker container $dockerContainer not found, building"
fi


# if the build container doesnt exist, build it
# if it does exist:
#		and no rebuild is set to true, then dont rebuild
#		and no rebuild is not set, then prompt to rebuild
if [[ $build_container_exists == true ]]; then

	if [[ $arg_no_docker_rebuild == true ]]; then
		rebuild_docker=false

	else
	
		echo
		echo "Build container $dockerContainer already exists, would you like to rebuild? [y/n] "
		echo 
		echo "[!!!] WARNING: THIS WILL DESTROY THE EXISTING BUILD CONTAINER AND REBUILD FROM SCRATCH"
		echo "                               THIS PROCESS CAN TAKE OVER 1 HOUR"
		echo
		read -p "    [y/n] --> " rebuildContainerYn

		if [[ $rebuildContainerYn =~ ^[yY]$ ]]; then
			echo
			read -p "    Are you sure you want to rebuild the docker container? [y/n] " rebuildContainerYnConfirm
			if [[ $rebuildContainerYnConfirm =~ ^[yY]$ ]]; then
				rebuild_docker=true
			else
				rebuild_docker=false
			fi
		else
			rebuild_docker=false
		fi
	fi

else
	rebuild_docker=true

fi

if [[ $rebuild_docker == false ]]; then
	echo
	echo "#### Skipping Docker Container Build ####"
else
	echo
	echo "#### Docker Container Build ####"
	
	if [ $build_container_exists == true ]; then
		echo "Removing old build containers"
		sudo docker rm -v $dockerContainer 
		sudo docker image rm -f $dockerContainer
	fi
	
	#if v2xhub plugin need to build image inside the V2X-Hub dir
	if $isV2xhubPlugin; then

		docker pull $dockerContainer
	

	#if we are vug-threads or protocolio
	elif [ $dockerContainer == "usdotfhwastoldev/voices:build-carla-latest" ]; then
		
		if [ ! -d $VUG_LOCAL_TENADEV_DIR/$vug_carla_adapter_name ]; then
			echo "This application uses the usdotfhwastoldev/voices:build-carla-latest build container from the $vug_carla_adapter_name. Cloning $vug_carla_adapter_name repository to use dockerfile"
		
			git clone $carlaTenaAdapterGitUrl -b develop $VUG_LOCAL_TENADEV_DIR/$vug_carla_adapter_name || exit

		fi

		dockerfileToUse=$VUG_LOCAL_TENADEV_DIR/$vug_carla_adapter_name/docker/Dockerfile

	elif [ $dockerContainer == "usdotfhwastoldev/voices:build-general-latest" ]; then

		dockerfileToUse=$localVoicesPocDir/scripts/build_tena_adapters/tena-general-dockerfile
	
	fi

	echo
	read -p "Would you like to use cached dockerfile steps? [y/n] " useDockerfileCache

	if [[ $useDockerfileCache =~ ^[yY]$ ]]; then

		dockerfileCacheArg=""

	else
	
		dockerfileCacheArg="--no-cache"
		
	fi

	echo
	echo "#### Starting Docker Build ####"
	sudo -E docker build $dockerfileCacheArg --force-rm --rm -f $dockerfileToUse -t $dockerContainer .
			
	currentDockerImages=$(sudo docker image list -q $dockerContainer)

	if [[ -z $currentDockerImages ]]; then
		echo
		echo "[!!!] Container $dockerContainer not built, check logs for details..."
		exit
	fi

	echo "#### Docker Build Complete ####"
		
fi


#-- Cmake example
#sudo docker run --rm -v /home/ejslattery/dev/carlaadapter:/home/CarlaAdapter -v /home/ejslattery/dev/tenadev/u1804-gcc75-64/TENA:/home/TENA usdotfhwastoldev/voices:build-carla-latest bash -c "cd /home/CarlaAdapter/build; export TENA_PLATFORM=u1804-gcc75-64; export TENA_HOME=/home/TENA; export TENA_VERSION=6.0.7; export CARLA_HOME=/home/carla; cmake -D CMAKE_EXPORT_COMPILE_COMMANDS=ON -D CMAKE_PREFIX_PATH=/home/TENA/lib/cmake -D BOOST_INCLUDEDIR=/home/TENA/TENA_boost_1.70.0.2_Library/u1804-gcc75-64/include -D VUG_INSTALL_DIR=/home/CarlaAdapter/INSTALL ../"
#-- Cmake

if [[ ! -d $localAppDir/build ]]; then
	mkdir $localAppDir/build
else 
	sudo rm -rf $localAppDir/build
	mkdir $localAppDir/build
fi

echo
echo "#### Running CMAKE ####"

#check for mw library
#ls $localTenaDir/lib/cmake
# if [[ ! -d $localTenaDir/lib/cmake/mw ]]; then
# 	echo
# 	echo mw library not installed in local TENA install $localTenaDir/lib/cmake/mw
# 	echo Pulling mw library
# 	sudo git clone https://github.com/usdot-fhwa-stol/vug-cmake-package.git cmake_temp || exit
# 	sudo mv cmake_temp/cmake/ $localTenaDir/lib/ || exit
# 	sudo rm -rf cmake_temp || exit
# fi

echo

if ! ( set -x ; sudo docker run --entrypoint /bin/bash --rm -v $localAppDir:$remoteAppDir  -v $localInstallDir:$remoteInstallDir $dockerContainer -c "cd $remoteAppDir/build; export TENA_PLATFORM=$tenaBuildVersion; export TENA_HOME=$remoteTenaDir; export TENA_VERSION=6.0.8; export CARLA_HOME=$remoteCarlaDir; cmake -D CMAKE_EXPORT_COMPILE_COMMANDS=ON $buildVersionDirArg $buildVersionCmakeArg -D CMAKE_PREFIX_PATH='$remoteTenaDir/lib/cmake;$remoteInstallDir;/opt/carma/cmake' -D VUG_INSTALL_DIR=$remoteInstallDir -D tmx-plugin_DIR=/usr/local/share/tmx/ ../" ); then
	echo
	echo "[!!!] CMAKE FAILED"
	exit
fi

echo
echo "#### CMAKE Complete ####"

#--Make example
#sudo docker run --rm -v /home/ejslattery/dev/carlaadapter:/home/CarlaAdapter -v /home/ejslattery/dev/tenadev/u1804-gcc75-64/TENA:/home/TENA usdotfhwastoldev/voices:build-carla-latest bash -c "cd /home/CarlaAdapter/build; export TENA_PLATFORM=u1804-gcc75-64; export TENA_HOME=/home/TENA; export TENA_VERSION=6.0.7; export CARLA_HOME=/home/carla; make VERBOSE=1"

#-- make
if [[ "$skipMake" == true ]]
	then
		echo
		echo "#### Skipping Make ####"
	else
		echo
		echo "#### Running Make ####"
		
		echo
		echo "MAKE COMMAND: "
		echo
		if ! ( set -x ; sudo docker run --entrypoint /bin/bash --rm -v $localAppDir:$remoteAppDir  -v $localInstallDir:$remoteInstallDir $dockerContainer -c "cd $remoteAppDir/build/$buildVersion; export TENA_PLATFORM=$tenaBuildVersion; export TENA_HOME=$remoteTenaDir; export TENA_VERSION=6.0.8; export CARLA_HOME=$remoteCarlaDir; make -j $numBuildJobs VERBOSE=1" ); then
			echo
			echo "[!!!] MAKE FAILED"
			exit
		fi

		echo
		echo "#### Make Complete ####"

		if $isV2xhubPlugin
			then
		
				echo
				echo "#### Running Make Package ####"
				
				echo
				echo "MAKE PACKAGE COMMAND: "
				if ! ( set -x ; sudo docker run --entrypoint /bin/bash --rm -v $localAppDir:$remoteAppDir  -v $localInstallDir:$remoteInstallDir $dockerContainer -c "cd $remoteAppDir/build/$buildVersion; export TENA_PLATFORM=$tenaBuildVersion; export TENA_HOME=$remoteTenaDir; export TENA_VERSION=6.0.8; export CARLA_HOME=$remoteCarlaDir; make -j $numBuildJobs package VERBOSE=1" ); then
					echo
					echo "[!!!] MAKE PACKAGE FAILED"
					exit
				fi

				
				echo
				echo "#### Make Package Complete ####"
			else
			
				echo
				echo "#### Running Make Install ####"
				
				echo
				echo "MAKE INSTALL COMMAND: "
				if ! ( set -x ; sudo docker run --entrypoint /bin/bash --rm -v $localAppDir:$remoteAppDir -v $localInstallDir:$remoteInstallDir $dockerContainer -c "cd $remoteAppDir/build/$buildVersion; export TENA_PLATFORM=$tenaBuildVersion; export TENA_HOME=$remoteTenaDir; export TENA_VERSION=6.0.8; export CARLA_HOME=/home/carla; make install VERBOSE=1" ); then
					echo
					echo "[!!!] MAKE INSTALL FAILED"
					exit
				fi

				echo
				echo "#### Make Install Complete ####"
		fi
fi

echo
echo
echo "#### Build Script Complete ####"
echo

