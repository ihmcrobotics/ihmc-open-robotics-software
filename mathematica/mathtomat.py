#!/usr/bin/python

# Author: Jordan Lack
# Date: 12/2013

# Feel free to download, distribute, and/or alter 
# as needed.

# This script takes a .mat file that has been outputted 
# by Mathematica, reads it in, and performs the necessary 
# expression substitutions to convert it to a file readable 
# by Matlab/Octave. I have not included the conversions for 
# ArcSine ect, but they can be added easily and may be added 
# in future versions or feel free to add them yourself. If you 
# do decide to revise this code and add updates/improvements, 
# please REPOST it online for others to use and learn from.

# In order to have Mathematica output the necessary file, 
# the following code snippet needs to be in your Mathematica 
# file. For example, if you have a Mathematica expression 
# called C that you want to convert to Matlab/Octave, after 
# the expression C has been evaluated in Mathematica, this 
# code snipped will output the necessary .mat file:

# stream = OpenWrite["C.mat"];
# Write[stream, C];
# Close[stream];
# Clear[stream];

# A few important things:
# 1. Make sure in your Mathematica notebook you have, SetDirectory[NotebookDirectory[]] right before the call to run this script, but not before the above snippet.
# 2. For each file expression you want to convert, you will need the four lines above.
# 3. The exact line to run this(so you can copy paste if you want) is: Run["python mathtomat.py"]

# Also note that this will make a Matlab/Octave function 
# that has only 1 argument; however, it should be straight forward 
# to add other arguments. Additionally, this assumes the .mat file is 
# in the /build directory(which should be a subdirectory of the directory you put 
# this file in). I have Mathematica put the .mat file directly in the /build folder; 
# however, you may choose move your .mat file there manually.

#import necessary modules
import re
import os
import glob

# Function that will do regex substitutions defined in dict on the string text
def multiSub(dict,text):
	# Function for performing multiple substitutions using regex

 	# Create a regular expression  from the dictionary keys
 	regex = re.compile("(%s)" % "|".join(map(re.escape, dict.keys())))

  	# For each match, look-up corresponding value in dictionary
  	return regex.sub(lambda mo: dict[mo.string[mo.start():mo.end()]], text) 

# Get the path to this file
baseDir = os.getcwd()

# This assumes your Mathematica expression files are in a subdirectory called "build"
buildDir = baseDir + '/build'

# Get the filepaths to all files in the build directory with a .mat extension
inputPaths = []
for files in glob.glob(buildDir+"/*.mat"):
	inputPaths.append(files)

# Create paths for output files corresponding to each input file
outputPaths = []
for files in inputPaths:
	tmpFile = files.split('/')[-1].split('.mat')[0]
	outputPaths.append(buildDir + '/' + tmpFile + '.m')

# Create dictionary of expressions to be substituted and what is to be put in their place
subDict = {"[":"(", "]":")", "{":"[", "}":"]"}

# loop through the filepaths
for indx,files in enumerate(inputPaths):
	# open the files as writable
	newFile  = open(outputPaths[indx], 'w+')

	# create the function definition text. Note that if you want to add more 
	# arguments, the (x) here is the argument, you could make this a list or 
	# generalize the way this is done to make it more flexible/general
	newFile.write("function" + " ret = " + files.split('/')[-1].split('.mat')[0] + "(x)\n\n")

	# write the Mathematica expression as well as make it all lowercase, make newlines 
	# compatible with Matlab/Octave and perform a somewhat hacky way of detecting the 
	# matrix rows and putting in the colons.
	contents = open(files).read().lower().replace('},','};').replace('\n','...\n')

	# Use the multiSub function to perform regex substitutions.
	newFile.write("ret = " + multiSub(subDict,contents).replace(" ","") + ";" + "\n\n" + "end")
	