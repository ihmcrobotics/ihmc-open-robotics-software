MYPATH=`readlink -f $0`
MYDIR=`dirname $MYPATH`/


THIRDPARTYJARS=$MYDIR/ThirdPartyJars/
CONFIGFILES=$MYDIR/config/
ROSDEPENDENCIES=$MYDIR/ROSDependencies/

if [ -d $THIRDPARTYJARS ]; then
	cd $THIRDPARTYJARS
	svn up
else 
	svn co https://bengal.ihmc.us/svn/RobotControl/ThirdParty/ThirdPartyJars/ $THIRDPARTYJARS
fi
cd $MYDIR

if [ -d $ROSDEPENDENCIES ]; then
	cd $ROSDEPENDENCIES
	svn up
else
	svn co	https://bengal.ihmc.us/svn/RobotControl/ROSJavaProjects/ROSDependencies/Jars/ $ROSDEPENDENCIES
fi
cd $MYDIR

if [ -d $CONFIGFILES ]; then
        cd $CONFIGFILES
        svn up
else
        svn co  https://bengal.ihmc.us/svn/RobotControl/DarpaRoboticsChallenge/config $CONFIGFILES
fi
cd $MYDIR


CLASSPATH=$(find $THIRDPARTYJARS $ROSDEPENDENCIES -name \*.jar -printf '%p:')DRCController.jar

