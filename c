[33mcommit f914b4489bee12a60efd69215855498465620028[m[33m ([m[1;36mHEAD -> [m[1;32mfeature/assistive-teleop[m[33m, [m[1;31morigin/feature/assistive-teleop[m[33m)[m
Merge: 2770b280602 58547cffc99
Author: Luigi Penco <lpenco@ihmc.org>
Date:   Fri Jun 9 15:26:43 2023 -0500

    merge with develop

[33mcommit 58547cffc99be32c83b9d6cb180d68d5f9dca61d[m
Author: Duncan Calvert <dcalvert@ihmc.us>
Date:   Fri Jun 9 11:59:52 2023 -0500

    Fix action sequence bugs.

[33mcommit 1be2b0c2d35960ce7961bcf793400b6818e6f0c0[m
Merge: dfbebac7c0d 7481adf5655
Author: Dexton Anderson <danderson@ihmc.org>
Date:   Fri Jun 9 15:31:25 2023 +0000

    Merge pull request #2576 in LIBS/ihmc-open-robotics-software from feature/promp-windows-container-build to develop
    
    * commit '7481adf565584752a02990bd6a2feff447780580':
      Update promp Linux lib
      Create Windows build container for promp

[33mcommit 2770b28060205026c0d1f44f075450395d9277d6[m
Author: Luigi Penco <lpenco@ihmc.org>
Date:   Thu Jun 8 18:21:14 2023 -0500

    added tracking chest and left elbow and vive trackers support in rdx

[33mcommit dfbebac7c0dd8a67a03c7a28fab3ae0f8c14ce02[m
Merge: 0771bbd8e02 56fe2e48664
Author: Tomasz Bialek <tbialek@ihmc.org>
Date:   Thu Jun 8 22:50:07 2023 +0000

    Merge pull request #2567 in LIBS/ihmc-open-robotics-software from bugfix/height-map-memory-leak to develop
    
    * commit '56fe2e48664b8c825b67422512c61c54bfb262ed':
      Fixed memory leak in ouster process

[33mcommit 7481adf565584752a02990bd6a2feff447780580[m
Author: Dexton Anderson <danderson@ihmc.org>
Date:   Thu Jun 8 16:41:23 2023 -0500

    Update promp Linux lib

[33mcommit dd6bb71da84827190840fbc4db2770faa1ed590f[m
Author: Dexton Anderson <danderson@ihmc.org>
Date:   Thu Jun 8 16:38:57 2023 -0500

    Create Windows build container for promp

[33mcommit 0771bbd8e02499c05a2d3b36086a57a04271f515[m[33m ([m[1;31morigin/feature/perception-gpu-height-map-rs-d455[m[33m)[m
Merge: b8b3fd6cddb 09c103b7b01
Author: Duncan Calvert <dcalvert@ihmc.us>
Date:   Thu Jun 8 21:36:18 2023 +0000

    Merge pull request #2574 in LIBS/ihmc-open-robotics-software from feature/automatic-action-sequence-sync to develop
    
    * commit '09c103b7b016e16e89646ed34b919e6394357ea8':
      Add set to reference frame for walk action.
      Set new walk goals to current mid feet Z up.
      Automatically synchronize the action sequence over the network.

[33mcommit b8b3fd6cddb447f0745e60661a0027d547eec364[m
Merge: 181cd5b228f 5b2a4cc1b13
Author: Duncan Calvert <dcalvert@ihmc.us>
Date:   Thu Jun 8 16:31:47 2023 +0000

    Merge pull request #2572 in LIBS/ihmc-open-robotics-software from feature/simplify-action-sequence-sync to develop
    
    * commit '5b2a4cc1b13b9209048512a99eb8725f55d8ddb5':
      Remove oops
      Simplify action sequence sync messages and fix loading order bug.

[33mcommit 09c103b7b016e16e89646ed34b919e6394357ea8[m
Author: Duncan Calvert <dcalvert@ihmc.us>
Date:   Thu Jun 8 11:30:11 2023 -0500

    Add set to reference frame for walk action.

[33mcommit 234c379c3ecbf17290c6ffd87a0b4a9d7f25c64d[m
Author: Duncan Calvert <dcalvert@ihmc.us>
Date:   Thu Jun 8 11:29:26 2023 -0500

    Set new walk goals to current mid feet Z up.

[33mcommit c4d68c24a88206f5ff28d88522494615d0037f87[m
Merge: 5122828ff03 181cd5b228f
Author: Duncan Calvert <dcalvert@ihmc.us>
Date:   Thu Jun 8 11:01:46 2023 -0500

    Merge branch 'develop' into feature/automatic-action-sequence-sync

[33mcommit 181cd5b228f264bc59c1a8a6c2ff1d8bd3c69506[m
Author: Duncan Calvert <dcalvert@ihmc.us>
Date:   Thu Jun 8 10:35:55 2023 -0500

    Revert gizmo change.

[33mcommit 5122828ff034ecfce2c7378fb210a883cac1bab5[m
Author: Duncan Calvert <dcalvert@ihmc.us>
Date:   Thu Jun 8 10:22:31 2023 -0500

    Automatically synchronize the action sequence over the network.

[33mcommit 5b2a4cc1b13b9209048512a99eb8725f55d8ddb5[m
Author: Duncan Calvert <dcalvert@ihmc.us>
Date:   Thu Jun 8 09:24:43 2023 -0500

    Remove oops

[33mcommit 3435479cbb53857c74030645aa65eeaa37c9eda1[m
Merge: 309d665c07a 487507dcac3
Author: Duncan Calvert <dcalvert@ihmc.us>
Date:   Thu Jun 8 14:23:37 2023 +0000

    Merge pull request #2571 in LIBS/ihmc-open-robotics-software from feature/action-sequence-authoring-improvements to develop
    
    * commit '487507dcac3931296fe29fb8ff5d256289f653a6':
      Fix action insertion UI. It's a lot less insane now.
      Gizmo improvements.
      Extract more.
      Extract stuff.
      Improve naming.
      Fix action insertion index.

[33mcommit 13dfe15d789506a838a374d42e83e3b714656092[m
Author: Duncan Calvert <dcalvert@ihmc.us>
Date:   Wed Jun 7 21:21:02 2023 -0500

    Simplify action sequence sync messages and fix loading order bug.

[33mcommit 309d665c07a84b5c9a3bd0bb94067958c70051e1[m
Merge: df3da0077a8 76a25a82c90
Author: Duncan Calvert <dcalvert@ihmc.us>
Date:   Thu Jun 8 00:27:31 2023 +0000

    Merge pull request #2565 in LIBS/ihmc-open-robotics-software from feature/action-frame-simplification to develop
    
    * commit '76a25a82c9095b60f3741dc06406a341bb90ffa2':
      Move frame closer to where it's used.
      Fix critical authoring bugs.
      Maintain JSON format ordering.
      Fix up frame code for footstep action.
      Cleanup frame code for interactable robot links.
      Unused imports
      Fix confusing bug from yesterday.
      Clean up hand pose action frame stuff.
      Cleanup goal feet transforms.
      Simplifying the reference frames for the walk action.
      Fix critical JSON loading bug that was breaking action sequences.

[33mcommit df3da0077a82fe1d26dcef45c5cd2a4e5521c647[m
Merge: 7f4eba19fe0 af10c75f28e
Author: Duncan Calvert <dcalvert@ihmc.us>
Date:   Wed Jun 7 22:27:57 2023 +0000

    Merge pull request #2570 in LIBS/ihmc-open-robotics-software from bugfix/gizmo-npe to develop
    
    * commit 'af10c75f28eb0a504fb47f02a1dfb920fc01b913':
      Gizmo improvements.

[33mcommit 7f4eba19fe01e988e3951ea78391d9ca7e2d49f3[m
Merge: f46abc9ac57 cc7ea3e83e6
Author: Duncan Calvert <dcalvert@ihmc.us>
Date:   Wed Jun 7 22:27:49 2023 +0000

    Merge pull request #2569 in LIBS/ihmc-open-robotics-software from bugfix/right-double-click-3d-panel to develop
    
    * commit 'cc7ea3e83e63aa308fd5b83ab118630ccb3b2afa':
      Double right click only enabled when hovering over the window.

[33mcommit 487507dcac3931296fe29fb8ff5d256289f653a6[m[33m ([m[1;31morigin/feature/action-sequence-authoring-improvements[m[33m)[m
Author: Duncan Calvert <dcalvert@ihmc.us>
Date:   Wed Jun 7 17:27:01 2023 -0500

    Fix action insertion UI. It's a lot less insane now.

[33mcommit af10c75f28eb0a504fb47f02a1dfb920fc01b913[m
Author: Duncan Calvert <dcalvert@ihmc.us>
Date:   Wed Jun 7 17:17:35 2023 -0500

    Gizmo improvements.

[33mcommit 1ab6e2785cea8939deba0bb0749c52342cf40e23[m
Author: Duncan Calvert <dcalvert@ihmc.us>
Date:   Wed Jun 7 17:17:35 2023 -0500

    Gizmo improvements.

[33mcommit f46abc9ac57f1f7867d0be5dc6c715ae9aabddcc[m
Author: Sylvain Bertrand <sbertrand@ihmc.org>
Date:   Wed Jun 7 17:10:43 2023 -0500

    Added weight for minimizing overall acceleration of each segment.

[33mcommit af79a1616681ed904ebe52d4fa640b7909547b41[m
Author: Duncan Calvert <dcalvert@ihmc.us>
Date:   Wed Jun 7 16:52:39 2023 -0500

    Extract more.

[33mcommit cf8ffc527cb0231f65f1d2dcaf612c63c9ab9880[m
Author: Duncan Calvert <dcalvert@ihmc.us>
Date:   Wed Jun 7 16:47:53 2023 -0500

    Extract stuff.

[33mcommit e5a99ed5b817079cb23ba80245963f7cf841ac34[m
Author: Duncan Calvert <dcalvert@ihmc.us>
Date:   Wed Jun 7 16:44:12 2023 -0500

    Improve naming.

[33mcommit 1291c5f1d8f1198db3cd20f551c630b2056c02af[m
Merge: a7a96acc7d3 ca0976deb49
Author: Duncan Calvert <dcalvert@ihmc.us>
Date:   Wed Jun 7 21:40:00 2023 +0000

    Merge pull request #2568 in LIBS/ihmc-open-robotics-software from bugfix/correct-rdx-documentation to develop
    
    * commit 'ca0976deb49013d6611adefae166509598a9cfc0':
      Correct RDX documentation on the order that things will be called.

[33mcommit cc7ea3e83e63aa308fd5b83ab118630ccb3b2afa[m
Author: Duncan Calvert <dcalvert@ihmc.us>
Date:   Wed Jun 7 16:39:09 2023 -0500

    Double right click only enabled when hovering over the window.

[33mcommit 2b4fddd34eea571e97c4376701435a03f4cceae9[m
Author: Duncan Calvert <dcalvert@ihmc.us>
Date:   Wed Jun 7 16:24:32 2023 -0500

    Fix action insertion index.

[33mcommit 76a25a82c9095b60f3741dc06406a341bb90ffa2[m[33m ([m[1;31morigin/feature/action-frame-simplification[m[33m)[m
Author: Duncan Calvert <dcalvert@ihmc.us>
Date:   Wed Jun 7 15:46:46 2023 -0500

    Move frame closer to where it's used.

[33mcommit 1410359638c09f60edf3ac4d35b16b38b996469a[m
Author: Duncan Calvert <dcalvert@ihmc.us>
Date:   Wed Jun 7 15:40:54 2023 -0500

    Fix critical authoring bugs.

[33mcommit ca0976deb49013d6611adefae166509598a9cfc0[m
Author: Duncan Calvert <dcalvert@ihmc.us>
Date:   Wed Jun 7 15:34:52 2023 -0500

    Correct RDX documentation on the order that things will be called.

[33mcommit cf4703ef9566901c8da9a56cf76ab3eae82c2ff6[m
Author: Duncan Calvert <dcalvert@ihmc.us>
Date:   Wed Jun 7 15:07:38 2023 -0500

    Maintain JSON format ordering.

[33mcommit 698cdd25aa1daa7feb9713ec67ca4325acaa60a9[m
Author: Duncan Calvert <dcalvert@ihmc.us>
Date:   Wed Jun 7 15:06:52 2023 -0500

    Fix up frame code for footstep action.

[33mcommit 56fe2e48664b8c825b67422512c61c54bfb262ed[m[33m ([m[1;31morigin/bugfix/height-map-memory-leak[m[33m)[m
Author: Tomasz Bialek <tbialek@ihmc.org>
Date:   Wed Jun 7 14:34:38 2023 -0500

    Fixed memory leak in ouster process

[33mcommit b82c36783d1419bc2a1db24fd68fa7a30ecaa43d[m
Author: Duncan Calvert <dcalvert@ihmc.us>
Date:   Wed Jun 7 14:24:14 2023 -0500

    Cleanup frame code for interactable robot links.

[33mcommit a7a96acc7d317c37b04f6c6a673339e51cda5473[m
Author: Dexton Anderson <danderson@ihmc.org>
Date:   Wed Jun 7 13:41:45 2023 -0500

    Fix slam-wrapper Dockerfile version

[33mcommit e32e1bed7dd8750a8716835a43125aad6c131bcd[m
Author: Dexton Anderson <danderson@ihmc.org>
Date:   Wed Jun 7 12:11:57 2023 -0500

    Update gtsam/slam-wrapper Windows libraries

[33mcommit 8a1fcc36bd669a79ae2bca0e607403ff90578b01[m
Author: Dexton Anderson <danderson@ihmc.org>
Date:   Wed Jun 7 12:09:43 2023 -0500

    Delete libmetis-gtsam.so

[33mcommit 05365f1a7a054269b07ceddc715c3291ae4248a9[m
Author: Dexton Anderson <danderson@ihmc.org>
Date:   Wed Jun 7 12:09:19 2023 -0500

    Update gtsam Linux libraries

[33mcommit 65d9010d9faa44517fcdf12cc720a10e545d6105[m
Author: Dexton Anderson <danderson@ihmc.org>
Date:   Wed Jun 7 11:53:29 2023 -0500

    No longer copy metis-gtsam.dll

[33mcommit 320b9675db7b70ca0c68d9fbf4f2dfae6b067e98[m
Author: Dexton Anderson <danderson@ihmc.org>
Date:   Wed Jun 7 11:53:03 2023 -0500

    Update gtsam Windows libraries

[33mcommit 632238c5ba2ec7b3e555ff4bb0a8bc025d3c3e11[m
Author: Dexton Anderson <danderson@ihmc.org>
Date:   Wed Jun 7 11:40:58 2023 -0500

    Use gtsam latest develop commit for slam-wrapper

[33mcommit 82c8d9ff27af94ac1126e048a53cf0149e35b46c[m
Author: Duncan Calvert <dcalvert@ihmc.us>
Date:   Wed Jun 7 11:01:10 2023 -0500

    Unused imports

[33mcommit da0781769b6ffb070b033ab313171e2c498ee4e9[m
Author: Duncan Calvert <dcalvert@ihmc.us>
Date:   Wed Jun 7 10:51:30 2023 -0500

    Fix confusing bug from yesterday.

[33mcommit e8c5ac7eea10418ba06b7c07fe42b86fdaceb2d7[m
Merge: a0f53f61770 5f602ba724c
Author: Dexton Anderson <danderson@ihmc.org>
Date:   Wed Jun 7 14:58:23 2023 +0000

    Merge pull request #2543 in LIBS/ihmc-open-robotics-software from feature/slam-wrapper-windows to develop
    
    * commit '5f602ba724c0eece793c864b547cb0e7897f8263':
      Formatting
      Working Windows slam-wrapper support
      slam-wrapper Windows updates
      Start slam-wrapper windows support

[33mcommit 5f602ba724c0eece793c864b547cb0e7897f8263[m
Merge: 65f0b7d5844 a0f53f61770
Author: Dexton Anderson <danderson@ihmc.org>
Date:   Wed Jun 7 09:57:43 2023 -0500

    Merge branch 'develop' into feature/slam-wrapper-windows

[33mcommit 6772e0604321b0fafbc7bb0e1f797949feb08076[m
Author: Duncan Calvert <dcalvert@ihmc.us>
Date:   Tue Jun 6 19:12:51 2023 -0500

    Clean up hand pose action frame stuff.

[33mcommit abcfeebb8c2bb14faa512adf1954da9464ed1d5f[m
Author: Duncan Calvert <dcalvert@ihmc.us>
Date:   Tue Jun 6 17:41:17 2023 -0500

    Cleanup goal feet transforms.

[33mcommit aeaf7f28b050433f0af5399c1c43f43b396bd7e9[m
Merge: 368b836a410 a0f53f61770
Author: Duncan Calvert <dcalvert@ihmc.us>
Date:   Tue Jun 6 17:28:33 2023 -0500

    Merge branch 'develop' into feature/action-frame-simplification

[33mcommit a0f53f617701a82ba4bb79de33869d1b13151f88[m
Author: Duncan Calvert <dcalvert@ihmc.us>
Date:   Tue Jun 6 16:49:41 2023 -0500

    Fix critical JSON loading bug that was breaking action sequences.

[33mcommit 368b836a410914e6e53c44a34857ca0e3518dbb8[m
Author: Duncan Calvert <dcalvert@ihmc.us>
Date:   Tue Jun 6 17:16:02 2023 -0500

    Simplifying the reference frames for the walk action.

[33mcommit 709cb76503a85bebc1dea0a2f53a3d3efed8d533[m
Author: Duncan Calvert <dcalvert@ihmc.us>
Date:   Tue Jun 6 16:49:41 2023 -0500

    Fix critical JSON loading bug that was breaking action sequences.

[33mcommit 18f2bd8b6b6b12c3e1ca16ebd02a722489a26b45[m
Merge: 7c8f9cba4ae afd6d137920
Author: Tomasz Bialek <tbialek@ihmc.org>
Date:   Tue Jun 6 20:36:27 2023 +0000

    Merge pull request #2563 in LIBS/ihmc-open-robotics-software from feature/replan-footstep-when-assume-flat-ground-change to develop
    
    * commit 'afd6d1379208bfce46b8f30ae79bdbcf80fa9347':
      Removed planner from control ring
      Made control ring and ball and arrow replan footsteps when state of assume flat ground is changed

[33mcommit 7c8f9cba4aedd30b5b3a0e0e57beeb49a5eb74f6[m
Merge: f193a26793a de822fb2131
Author: Duncan Calvert <dcalvert@ihmc.us>
Date:   Tue Jun 6 20:16:52 2023 +0000

    Merge pull request #2560 in LIBS/ihmc-open-robotics-software from feature/ui-construction-cleanup to develop
    
    * commit 'de822fb21317cb12c6118d269e500b8e3ffca40d':
      Finishing touches with Bhavyansh.
      Fix compile errors.
      Replace with Bhavyansh's class.
      Create planar region extractor tools again.
      Create a parameters class to pass camera parameters around with less boilerplate.
      Make the RapidPlanarRegionsExtractor constructor do the create.
      Extract construction of planar regions extractor.

[33mcommit de822fb21317cb12c6118d269e500b8e3ffca40d[m
Author: Duncan Calvert <dcalvert@ihmc.us>
Date:   Tue Jun 6 15:03:49 2023 -0500

    Finishing touches with Bhavyansh.

[33mcommit 817be6d550670799c78b4f6f5a621de6702b8def[m
Author: Duncan Calvert <dcalvert@ihmc.us>
Date:   Tue Jun 6 13:55:02 2023 -0500

    Fix compile errors.

[33mcommit df376e4c13e1d7be5ffb30a75475d717d38cfc87[m
Author: Duncan Calvert <dcalvert@ihmc.us>
Date:   Tue Jun 6 13:51:44 2023 -0500

    Replace with Bhavyansh's class.

[33mcommit afd6d1379208bfce46b8f30ae79bdbcf80fa9347[m
Author: Tomasz Bialek <tbialek@ihmc.org>
Date:   Tue Jun 6 13:42:14 2023 -0500

    Removed planner from control ring

[33mcommit 3bf05f6006e716f8885f28e6eecc9e76df8e6aaf[m
Author: Duncan Calvert <dcalvert@ihmc.us>
Date:   Tue Jun 6 13:40:13 2023 -0500

    Create planar region extractor tools again.

[33mcommit 094b83a1ed44203e39dfe97e1e00df379d1d8efb[m
Author: Duncan Calvert <dcalvert@ihmc.us>
Date:   Tue Jun 6 13:37:12 2023 -0500

    Create a parameters class to pass camera parameters around with less boilerplate.

[33mcommit 8074760fcd9e1f3dd0ac18fb8c89c06e364af38e[m
Author: Duncan Calvert <dcalvert@ihmc.us>
Date:   Tue Jun 6 13:24:49 2023 -0500

    Make the RapidPlanarRegionsExtractor constructor do the create.

[33mcommit f193a26793a7128cfdcaa5ac60ac92fc14775a1f[m
Merge: afddb062035 cd27360f015
Author: Tomasz Bialek <tbialek@ihmc.org>
Date:   Tue Jun 6 18:02:31 2023 +0000

    Merge pull request #2561 in LIBS/ihmc-open-robotics-software from feature/control-ring-snaps-to-height to develop
    
    * commit 'cd27360f0155cdb91033e5c054b3278f3593f2f2':
      Made control ring snap to height based on cursor position

[33mcommit 204dec6747a5c63a98249137dd81b619151558fa[m
Author: Tomasz Bialek <tbialek@ihmc.org>
Date:   Tue Jun 6 11:59:11 2023 -0500

    Made control ring and ball and arrow replan footsteps when state of assume flat ground is changed
