#pragma once

#include "vector"
#include "string"

class FileTools
{
    public:
        static void getFileNames(std::string dirName, std::vector<std::string>& files, bool printList = false);
};