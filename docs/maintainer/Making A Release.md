# Making A Release

## Develop branch

1. Update version numbers.
1. Create Release Notes.
1. Update README.
1. Update documentation. (When we have versioned documentation)

## Preparing for release

1. `develop` branch is green on Bamboo for all supported software.
1. Record the CI URLs for ihmc-open-robotics-software and record them the README.
   * There is a new  Status Icons plugin that gives an icon.
1. Pin the Atlas OCU to the global build number. Some tags (i.e. `build-3344`)will help here.
1. Clone only the repos needed to run Atlas. Should be around 5.
    * Make sure there are no other repos in your group.

## Atlas Testing

1. Atlas does obstacle course, all major features tested on real hardware.
   * https://confluence.ihmc.us/display/HOWTO/Checklist+for+Stable+atlas-develop+Tags
1. Update Release Notes on what was tested and performance results.
   * List things that were tested
   * Include log summary CSV
   * Upload log video to YouTube and link

## Releasing
1. Start release with git flow.
1. Run `gradle --console=plain --continue compositeTask -PtaskName=publish -PpublishMode=STABLE`
1. Verify publish on Bintray, approve it.
1. If publish fails, increment hotfix digit and try again. (e.g. `0.11.X`)
1. Tag the release with the version number and a simple annotated message.
1. Finish release with git flow.

## Polishing Release

1. Update GitHub release notes with RELEASE_NOTES.md (or something)



   

