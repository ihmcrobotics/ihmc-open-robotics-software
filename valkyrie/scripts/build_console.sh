#!/bin/bash
# Build the IHMC software on console. This includes deploying build products to
# link and zelda.

IHMC_HOME=$HOME/git/ihmc-open-robotics-software
GRADLE_VERSION=5.3.1
GRADLE=/usr/lib/gradle/${GRADLE_VERSION}/bin/gradle

if [[ ! -x $GRADLE ]]; then
    echo "Please install gradle ${GRADLE_VERSION}"
    echo "This may be done either by running the ihmc_open_source_robotics.yml playbook in val_develop"
    echo "or manually, by adding the PPA ppa:cwchien/gradle and installing the package gradle-${GRADLE_VERSION}"
    exit 1
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
$GRADLE --no-build-cache deploy
cd ..
