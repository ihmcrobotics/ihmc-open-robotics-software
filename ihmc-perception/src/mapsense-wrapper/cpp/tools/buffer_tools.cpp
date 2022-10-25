
#include "buffer_tools.h"

void BufferTools::PrintMatR8(cv::Mat& mat, int value, bool invert, bool constant, int rowLimit, int colLimit)
{
   int rows = rowLimit;
   int cols = colLimit;
   if (rowLimit == 0)
      rows = mat.rows;
   if (colLimit == 0)
      cols = mat.cols;

   for (int i = 0; i < rows; i++)
   {
      for (int j = 0; j < cols; j++)
      {
         uint8_t current = mat.at<uint8_t>(i, j);

         if(constant)
         {
             if(current > (uint8_t)value)
                 printf("1 ");
             else
                 printf("0 ");
         }
         else
         {
             printf("%hhu ", current);
         }

      }
      printf("\n", i);
   }
}

void BufferTools::PrintMatR16(cv::Mat& mat, int value, bool invert, int rowLimit, int colLimit, bool linear)
{
   int rows = rowLimit;
   int cols = colLimit;
   if (rowLimit == 0)
      rows = mat.rows;
   if (colLimit == 0)
      cols = mat.cols;

   if(!linear)
   {
      for (int i = 0; i < rows; i++)
      {
         for (int j = 0; j < cols; j++)
         {
            uint16_t current = mat.at<uint16_t>(i, j);
            printf("%hu ", current);
         }
         printf("\n");
      }
   } else
   {
      for (int i = 0; i < rows; i++)
      {
         for (int j = 0; j < cols; j++)
         {
            uint16_t current = mat.at<uint16_t>(i, j);
            printf("(%d %d): %.2lf\n", i, j, current);
         }
         printf("\n");
      }
   }
}
