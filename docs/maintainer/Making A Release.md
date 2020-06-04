# Making A Release

#### Generate messages

1. Run `us.ihmc.idl.IHMCInterfacesGenerateMessages`, 
   located in `ihmc-interfaces\src\generator\java`
   with `ihmc-open-robotics-software/ihmc-interfaces` set as the working directory.
2. Update the standalone `ihmc_msgs` and `ihmc_interfaces` repositories on GitHub.

#### Step Develop branch

1. Update version numbers.
1. Document all features, API changes, regressions, bug fixes, etc in `docs/release-notes/X.X Release Notes.md`
1. Update README.
1. Update documentation. (When we have versioned documentation)

#### Preparing for release

1. `develop` branch is green on Bamboo for all supported software.
1. Record the CI URLs for ihmc-open-robotics-software and record them the README.
   * There is a new  Status Icons plugin that gives an icon.
1. Pin the Atlas OCU to the global build number. Some tags (i.e. `build-3344`)will help here.
1. Clone only the repos needed to run Atlas. Should be around 5.
    * Make sure there are no other repos in your group.

#### Atlas Testing

1. Atlas does obstacle course, all major features tested on real hardware.
   * https://confluence.ihmc.us/display/HOWTO/Checklist+for+Stable+atlas-develop+Tags
1. Update Release Notes on what was tested and performance results.
   * List things that were tested
   * Include log summary CSV
   * Upload log video to YouTube and link

#### Releasing
1. Start release with git flow
1. Run `gradle compositePublish -PpublishUrl=ihmcRelease`
1. Verify publish on Bintray, approve it
1. If publish fails, increment hotfix digit and try again. (e.g. `0.11.X`)
1. Tag the release with the version number and a simple annotated message
1. Finish release with git flow

#### Step 6: Draft and publish release on GitHub

1. Go to [https://github.com/ihmcrobotics/ihmc-open-robotics-software/releases](https://github.com/ihmcrobotics/ihmc-open-robotics-software/releases)
1. Click "Draft a new release"
1. Enter version X.X.X as the tag name
1. Title the release "X.X.X Release Notes"
1. Copy the contents of X.X Release Notes.md
1. Click "Publish release"
