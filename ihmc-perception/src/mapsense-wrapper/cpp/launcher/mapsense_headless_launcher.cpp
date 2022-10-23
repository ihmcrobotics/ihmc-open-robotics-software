#include "mapsense_headless_launcher.h"

MapsenseHeadlessLauncher::MapsenseHeadlessLauncher(int argc, char **argv)
{
   _openCLManager = new OpenCLManager("");
   _regionCalculator = new PlanarRegionCalculator(argc, argv, appState);
   _regionCalculator->setOpenCLManager(_openCLManager);
}

void MapsenseHeadlessLauncher::update()
{
   // _regionCalculator->generateRegionsFromDepth(appState);
}

int main(int argc, char **argv)
{
   MapsenseHeadlessLauncher mapsense(argc, argv);

   std::vector<std::string> args(argv, argv + argc);

   for (int i = 0; i < argc; i++)
   {
      if (args[i] == "--export")
      {
         printf("Setting EXPORT_REGIONS: true\n");
         mapsense.appState.EXPORT_REGIONS = true;
      }
      if (args[i] == "--kernel-level")
      {
         printf("Setting KERNEL_LEVEL_SLIDER: %d\n", stoi(args[i + 1]));
         mapsense.appState.KERNEL_SLIDER_LEVEL = stoi(args[i + 1]);
      }
      if (args[i] == "--depth-aligned")
      {
         printf("Setting DEPTH_ALIGNED: true\n");
         mapsense.appState.DEPTH_ALIGNED = true;
      }
   }

   while (mapsense.appState.ROS_ENABLED)
   {
      mapsense.update();
   }
}