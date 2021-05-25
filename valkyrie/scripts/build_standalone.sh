#!/bin/bash
# Build the IHMC software for a standalone configuration. This should not be used on console since
# it deploys build products only on the local machine.

IHMC_HOME=$HOME/git/ihmc-open-robotics-software
GRADLE_VERSION=5.6.4
GRADLE=/usr/lib/gradle/${GRADLE_VERSION}/bin/gradle

if [[ ! -x $GRADLE ]]; then
    echo "Please install gradle ${GRADLE_VERSION}"
    echo "This may be done either by running the ihmc_open_source_robotics.yml playbook in val_develop"
    echo "or manually, by adding the PPA ppa:cwchien/gradle and installing the package gradle-${GRADLE_VERSION}"
    exit 0
fi

if [[ ! -d $IHMC_HOME ]]; then
    echo "Unable to find the IHMC source at $IHMC_HOME"
    exit 2
fi

cd $IHMC_HOME
$GRADLE --no-build-cache clean
$GRADLE --no-build-cache compositeTask -PtaskName=publishToMavenLocal -PcompositeSearchHeight=0 -PpublishMode=LOCAL
cd valkyrie
$GRADLE --no-build-cache clean
$GRADLE --no-build-cache deployLocal
cd ..
