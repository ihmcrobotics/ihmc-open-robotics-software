CP=../../../../ThirdParty/ThirdPartyJars/EJML/EJML.jar:jna-4.1.0.jar:.
all:
	javac -cp ${CP} Test.java 
	#javah -cp ${CP} Test
	#icpc -no-use-clang-env -cxxlib=/usr/local -gxx-name=g++-4.9  -O3 -fPIC -I /System/Library/Frameworks/JavaVM.framework/Versions/A/Headers `pkg-config --cflags eigen3`  -dynamiclib  Test.cpp -o libTest.jnilib
	g++  -Ofast -march=native -mtune=native -mfpmath=sse -msse3 -DNDEBUG  -fPIC -I /System/Library/Frameworks/JavaVM.framework/Versions/A/Headers `pkg-config --cflags eigen3`  Test.cpp -shared -o libTest.dylib
	java -cp  ${CP} Test
