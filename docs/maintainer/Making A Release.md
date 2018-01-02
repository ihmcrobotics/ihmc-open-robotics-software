# Making A Release

## Develop branch

Version number, release notes, documentation, and README should always target the next release.

## Preparing for release (Atlas certified)

1. `develop` branch is green on Bamboo for all supported software.
1. Record the CI URLs for ihmc-open-robotics-software and record them the README.
   * There is a new  Status Icons plugin that gives an icon.
1. Pin the Atlas OCU to the global build number. Some tags (i.e. `build-3344`)will help here.
1. Atlas does obstacle course, all major features tested on real hardware.

## Releasing
1. Start release with git flow.
1. Run `gradle publish -PdepthFromWorkspaceDirectory=0 -PpublishMode=STABLE`
1. Verify publish on Bintray, approve it.
1. If publish fails, increment hotfix digit and try again. (e.g. `0.11.X`)
1. Tag the release with the version number and a simple annotated message.
1. Finish release with git flow.

## Polishing Release

1. Update GitHub release notes. With RELEASE_NOTES.md (or something)



   

