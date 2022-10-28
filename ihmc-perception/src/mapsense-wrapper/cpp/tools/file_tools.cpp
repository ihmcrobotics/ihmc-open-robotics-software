#include "file_tools.h"

#include <dirent.h>
#include <algorithm>

void FileTools::getFileNames(std::string dirName, std::vector<std::string>& files, bool printList)
{
   if (auto dir = opendir(dirName.c_str()))
   {
      while (auto f = readdir(dir))
      {

         if (!f->d_name || f->d_name[0] == '.')
            continue;
         files.emplace_back(f->d_name);
      }
      closedir(dir);
   }
   sort(files.begin(), files.end());

   if (printList)
   {
      printf("[");
      for (std::string file : files)
      {
         printf("%s, ", file.c_str());
      }
      printf("]\n");
   }
}