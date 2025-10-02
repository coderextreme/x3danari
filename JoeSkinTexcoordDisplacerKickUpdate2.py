print('<!--')
import x3d
print('-->')
X3D0 = x3d.X3D()
X3D0.profile = "Immersive"
X3D0.version = "4.0"
head1 = x3d.head()
meta2 = x3d.meta()
meta2.name = "title"
meta2.content = "JoeSkinTexcoordDisplacerKick.x3d"

head1.children.append(meta2)
meta3 = x3d.meta()
meta3.name = "info"
meta3.content = "Joe No Reservations 20200709 fix hier20161206 20161111 20160720 20121221 20040109 x3d/hanim"

head1.children.append(meta3)
meta4 = x3d.meta()
meta4.name = "description"
meta4.content = "The Joe model is a Humanoid with textured skin."

head1.children.append(meta4)
meta5 = x3d.meta()
meta5.name = "creator"
meta5.content = "Joe Williams"

head1.children.append(meta5)
meta6 = x3d.meta()
meta6.name = "created"
meta6.content = "9 January 2014"

head1.children.append(meta6)
meta7 = x3d.meta()
meta7.name = "translated"
meta7.content = "12 January 2017"

head1.children.append(meta7)
meta8 = x3d.meta()
meta8.name = "modified"
meta8.content = "Tue, 26 Aug 2025 02:48:21 GMT"

head1.children.append(meta8)
meta9 = x3d.meta()
meta9.name = "TODO"
meta9.content = "Record information about skin coordinates (found in comment at end of scene) as a structured MetadataSet containing MetadataString nodes"

head1.children.append(meta9)
meta10 = x3d.meta()
meta10.name = "translators"
meta10.content = "Roy Walmsley and Don Brutzman"

head1.children.append(meta10)
meta11 = x3d.meta()
meta11.name = "identifier"
meta11.content = "https://www.web3d.org/x3d/content/examples/Basic/HumanoidAnimation/JoeSkinTexcoordDisplacerKick.x3d"

head1.children.append(meta11)
component12 = x3d.component()
component12.name = "HAnim"
component12.level = 3

head1.children.append(component12)

X3D0.head = head1
Scene13 = x3d.Scene()
WorldInfo14 = x3d.WorldInfo()
WorldInfo14.title = "X3D HANIM LOA3 Skeleton, 390 point Skin, texcoords, Displacer, teTrans for Joe_ by Joe"
WorldInfo14.info = ["X3D Humanoid V1 LOA3 skeleton","skin from hanim sites, surface features, and some added points","390 points"]

Scene13.children.append(WorldInfo14)
NavigationInfo15 = x3d.NavigationInfo()
NavigationInfo15.DEF = "Start_NavigationInfo"
NavigationInfo15.speed = 2.5
NavigationInfo15.headlight = False

Scene13.children.append(NavigationInfo15)
Background16 = x3d.Background()
Background16.DEF = "blue_Background"

Scene13.children.append(Background16)
SpotLight17 = x3d.SpotLight()
SpotLight17.DEF = "light1"
SpotLight17.color = [0.8,0.8,1]
SpotLight17.ambientIntensity = 0.7
SpotLight17.location = [0,3,3]
SpotLight17.direction = [0,0,0]
SpotLight17.radius = 10
SpotLight17.beamWidth = 1.5
SpotLight17.cutOffAngle = 0.6

Scene13.children.append(SpotLight17)
PointLight18 = x3d.PointLight()
PointLight18.DEF = "light2"
PointLight18.color = [0.8,0.8,1]
PointLight18.ambientIntensity = 0.7
PointLight18.location = [0,10,-7]

Scene13.children.append(PointLight18)
Viewpoint19 = x3d.Viewpoint()
Viewpoint19.DEF = "Scene_InclinedView"
Viewpoint19.description = "Scene_Inclined View"
Viewpoint19.position = [1.62,1.05,3.06]
Viewpoint19.orientation = [-0.113,0.993,0.0347,0.671]
Viewpoint19.centerOfRotation = [0,0.85,0]

Scene13.children.append(Viewpoint19)
Viewpoint20 = x3d.Viewpoint()
Viewpoint20.DEF = "Scene_IFrontView"
Viewpoint20.description = "Scene_Front View"
Viewpoint20.position = [0,0.8,2.58]
Viewpoint20.centerOfRotation = [0,0.8,0]

Scene13.children.append(Viewpoint20)
Viewpoint21 = x3d.Viewpoint()
Viewpoint21.DEF = "Scene_ISideView"
Viewpoint21.description = "Scene_Side View"
Viewpoint21.position = [2.6,0.5,0]
Viewpoint21.orientation = [0,1,0,1.5708]
Viewpoint21.centerOfRotation = [0,0.8,0]

Scene13.children.append(Viewpoint21)
Viewpoint22 = x3d.Viewpoint()
Viewpoint22.DEF = "Scene_BackView"
Viewpoint22.description = "Scene_Back View"
Viewpoint22.position = [0,2.5,-3]
Viewpoint22.orientation = [0,1,0,3.14]
Viewpoint22.centerOfRotation = [0,1.5,0]

Scene13.children.append(Viewpoint22)
Viewpoint23 = x3d.Viewpoint()
Viewpoint23.DEF = "Scene_TopView"
Viewpoint23.description = "Scene_Top View"
Viewpoint23.position = [0,3.5,0]
Viewpoint23.orientation = [1,0,0,-1.5708]
Viewpoint23.centerOfRotation = [0,1.5,0]

Scene13.children.append(Viewpoint23)
Group24 = x3d.Group()
Group24.DEF = "Joe_Humanoid"
HAnimHumanoid25 = x3d.HAnimHumanoid()
HAnimHumanoid25.DEF = "Joe_Human"
HAnimHumanoid25.name = "Human"
HAnimJoint26 = x3d.HAnimJoint()
HAnimJoint26.DEF = "Joe_HumanoidRoot"
HAnimJoint26.name = "HumanoidRoot"
HAnimJoint26.translation = [0.3747319,-0.0339008,-0.3825737]
HAnimJoint26.rotation = [1,0,0,0.265147406382152]
HAnimJoint26.center = [0,0.875,0]
HAnimSegment27 = x3d.HAnimSegment()
HAnimSegment27.DEF = "Joe_sacrum"
HAnimSegment27.name = "sacrum"
HAnimSite28 = x3d.HAnimSite()
HAnimSite28.DEF = "Joe_RootFront"
HAnimSite28.name = "RootFront"
Transform29 = x3d.Transform()
Transform29.DEF = "hanimcordsys"
Transform29.scale = [0.175,0.175,0.175]
Viewpoint30 = x3d.Viewpoint()
Viewpoint30.DEF = "ViewBodyRootAxes"
Viewpoint30.description = "Joe_HAnim Root Coordinate Axes View"

Transform29.children.append(Viewpoint30)
Shape31 = x3d.Shape()
Shape31.DEF = "AxisLinesShape"
IndexedLineSet32 = x3d.IndexedLineSet()
IndexedLineSet32.colorPerVertex = False
IndexedLineSet32.colorIndex = [0,1,2]
IndexedLineSet32.coordIndex = [0,1,-1,0,2,-1,0,3,-1]
Color33 = x3d.Color()
Color33.color = [(1, 0, 0),(0, 0.6, 0),(0, 0, 1)]

IndexedLineSet32.color = Color33
Coordinate34 = x3d.Coordinate()
Coordinate34.point = [(0, 0, 0),(1, 0, 0),(0, 1, 0),(0, 0, 1)]

IndexedLineSet32.coord = Coordinate34

Shape31.geometry = IndexedLineSet32

Transform29.children.append(Shape31)

HAnimSite28.children.append(Transform29)

HAnimSegment27.children.append(HAnimSite28)

HAnimJoint26.children.append(HAnimSegment27)
HAnimJoint35 = x3d.HAnimJoint()
HAnimJoint35.DEF = "Joe_sacroiliac"
HAnimJoint35.name = "sacroiliac"
HAnimJoint35.center = [0,0.92,0]
HAnimJoint35.skinCoordIndex = [17,19,20,21,22,23,26,27,73,82,89,91,93]
HAnimJoint35.skinCoordWeight = [1,1,1,1,1,1,1,1,1,1,0.35,0.35,1]
HAnimJoint36 = x3d.HAnimJoint()
HAnimJoint36.DEF = "Joe_l_hip"
HAnimJoint36.name = "l_hip"
HAnimJoint36.rotation = [-0.999999999999997,0,0,0.412868537827733]
HAnimJoint36.center = [0.1,0.92,0]
HAnimJoint36.skinCoordIndex = [89,90,94,95,96,97]
HAnimJoint36.skinCoordWeight = [0.65,1,1,1,1,1]
HAnimJoint37 = x3d.HAnimJoint()
HAnimJoint37.DEF = "Joe_l_knee"
HAnimJoint37.name = "l_knee"
HAnimJoint37.rotation = [0,0,1.00000000000037,0.0232350641430369]
HAnimJoint37.center = [0.115,0.466,0]
HAnimJoint37.skinCoordIndex = [334,335,336,337,338,339,340,341]
HAnimJoint37.skinCoordWeight = [1,1,1,1,1,1,1,1]
HAnimJoint38 = x3d.HAnimJoint()
HAnimJoint38.DEF = "Joe_l_ankle"
HAnimJoint38.name = "l_ankle"
HAnimJoint38.rotation = [-1.00000000000001,0,0,0.130294799804686]
HAnimJoint38.center = [0.115,0.069,0]
HAnimJoint38.skinCoordIndex = [342,343,344,345]
HAnimJoint38.skinCoordWeight = [1,1,1,1]
HAnimJoint39 = x3d.HAnimJoint()
HAnimJoint39.DEF = "Joe_l_subtalar"
HAnimJoint39.name = "l_subtalar"
HAnimJoint39.center = [0.115,0.031,0.03]
HAnimJoint39.skinCoordIndex = [346,347,348,71]
HAnimJoint39.skinCoordWeight = [1,1,1,1]
HAnimJoint40 = x3d.HAnimJoint()
HAnimJoint40.DEF = "Joe_l_midtarsal"
HAnimJoint40.name = "l_midtarsal"
HAnimJoint40.center = [0.115,0.037,0.09]
HAnimJoint40.skinCoordIndex = [349,350,351,352]
HAnimJoint40.skinCoordWeight = [1,1,1,1]
HAnimJoint41 = x3d.HAnimJoint()
HAnimJoint41.DEF = "Joe_l_metatarsal"
HAnimJoint41.name = "l_metatarsal"
HAnimJoint41.center = [0.115,0.02,0.122]
HAnimJoint41.skinCoordIndex = [353,354,355,356,357,358,359,360,361]
HAnimJoint41.skinCoordWeight = [1,1,1,1,1,1,1,1,1]

HAnimJoint40.children.append(HAnimJoint41)

HAnimJoint39.children.append(HAnimJoint40)

HAnimJoint38.children.append(HAnimJoint39)

HAnimJoint37.children.append(HAnimJoint38)

HAnimJoint36.children.append(HAnimJoint37)

HAnimJoint35.children.append(HAnimJoint36)
HAnimJoint42 = x3d.HAnimJoint()
HAnimJoint42.DEF = "Joe_r_hip"
HAnimJoint42.name = "r_hip"
HAnimJoint42.rotation = [1,0,0,1]
HAnimJoint42.center = [-0.1,0.92,0]
HAnimJoint42.skinCoordIndex = [91,92,98,99,100,101,362,363]
HAnimJoint42.skinCoordWeight = [0.65,1,0.8,1,1,1,0.4,0.8]
HAnimJoint43 = x3d.HAnimJoint()
HAnimJoint43.DEF = "Joe_r_knee"
HAnimJoint43.name = "r_knee"
HAnimJoint43.rotation = [0.999999999996814,0,0,0.0130294799805103]
HAnimJoint43.center = [-0.05,0.466,0]
HAnimJoint43.skinCoordIndex = [362,363,364,365,366,367,368,369,98]
HAnimJoint43.skinCoordWeight = [0.6,0.2,1,1,1,1,1,1,0.2]
HAnimJoint44 = x3d.HAnimJoint()
HAnimJoint44.DEF = "Joe_r_ankle"
HAnimJoint44.name = "r_ankle"
HAnimJoint44.rotation = [-1.00000000000001,0,0,0.130294799804686]
HAnimJoint44.center = [-0.115,0.069,0]
HAnimJoint44.skinCoordIndex = [370,371,372,373]
HAnimJoint44.skinCoordWeight = [1,1,1,1]
HAnimJoint45 = x3d.HAnimJoint()
HAnimJoint45.DEF = "Joe_r_subtalar"
HAnimJoint45.name = "r_subtalar"
HAnimJoint45.center = [-0.1,0.015,-0.01]
HAnimJoint45.skinCoordIndex = [374,375,376]
HAnimJoint45.skinCoordWeight = [1,1,1]
HAnimJoint46 = x3d.HAnimJoint()
HAnimJoint46.DEF = "Joe_r_midtarsal"
HAnimJoint46.name = "r_midtarsal"
HAnimJoint46.center = [-0.115,0.037,0.09]
HAnimJoint46.skinCoordIndex = [377,378,379,380]
HAnimJoint46.skinCoordWeight = [1,1,1,1]
HAnimJoint47 = x3d.HAnimJoint()
HAnimJoint47.DEF = "Joe_r_metatarsal"
HAnimJoint47.name = "r_metatarsal"
HAnimJoint47.center = [-0.1,0.01,0.14]
HAnimJoint47.skinCoordIndex = [381,382,383,384,385,386,387,388,389]
HAnimJoint47.skinCoordWeight = [1,1,1,1,1,1,1,1,1]

HAnimJoint46.children.append(HAnimJoint47)

HAnimJoint45.children.append(HAnimJoint46)

HAnimJoint44.children.append(HAnimJoint45)

HAnimJoint43.children.append(HAnimJoint44)

HAnimJoint42.children.append(HAnimJoint43)

HAnimJoint35.children.append(HAnimJoint42)

HAnimJoint26.children.append(HAnimJoint35)
HAnimJoint48 = x3d.HAnimJoint()
HAnimJoint48.DEF = "Joe_vl5"
HAnimJoint48.name = "vl5"
HAnimJoint48.center = [0,1.045,-0.095]
HAnimJoint48.skinCoordIndex = [28,76]
HAnimJoint48.skinCoordWeight = [1,1]
HAnimJoint49 = x3d.HAnimJoint()
HAnimJoint49.DEF = "Joe_vl4"
HAnimJoint49.name = "vl4"
HAnimJoint49.center = [0,1.068,-0.085]
HAnimJoint50 = x3d.HAnimJoint()
HAnimJoint50.DEF = "Joe_vl3"
HAnimJoint50.name = "vl3"
HAnimJoint50.center = [0,1.092,-0.0725]
HAnimJoint51 = x3d.HAnimJoint()
HAnimJoint51.DEF = "Joe_vl2"
HAnimJoint51.name = "vl2"
HAnimJoint51.center = [0,1.12,-0.065]
HAnimJoint51.skinCoordIndex = [16,18,25,83,84,85,86,87,88]
HAnimJoint51.skinCoordWeight = [1,1,1,1,1,1,0.7,1,0.8]
HAnimJoint52 = x3d.HAnimJoint()
HAnimJoint52.DEF = "Joe_vl1"
HAnimJoint52.name = "vl1"
HAnimJoint52.center = [0,1.1459,-0.0625]
HAnimJoint53 = x3d.HAnimJoint()
HAnimJoint53.DEF = "Joe_vt12"
HAnimJoint53.name = "vt12"
HAnimJoint53.center = [0,1.179,-0.068]
HAnimJoint54 = x3d.HAnimJoint()
HAnimJoint54.DEF = "Joe_vt11"
HAnimJoint54.name = "vt11"
HAnimJoint54.center = [0,1.2679,-0.081]
HAnimJoint55 = x3d.HAnimJoint()
HAnimJoint55.DEF = "Joe_vt10"
HAnimJoint55.name = "vt10"
HAnimJoint55.center = [0,1.242,-0.09]
HAnimJoint55.skinCoordIndex = [15]
HAnimJoint55.skinCoordWeight = [1]
HAnimJoint56 = x3d.HAnimJoint()
HAnimJoint56.DEF = "Joe_vt9"
HAnimJoint56.name = "vt9"
HAnimJoint56.center = [0,1.268,-0.1]
HAnimJoint56.skinCoordIndex = [13,14]
HAnimJoint56.skinCoordWeight = [1,1]
HAnimJoint57 = x3d.HAnimJoint()
HAnimJoint57.DEF = "Joe_vt8"
HAnimJoint57.name = "vt8"
HAnimJoint57.center = [0,1.294,-0.11]
HAnimJoint58 = x3d.HAnimJoint()
HAnimJoint58.DEF = "Joe_vt7"
HAnimJoint58.name = "vt7"
HAnimJoint58.center = [0,1.323,-0.1155]
HAnimJoint59 = x3d.HAnimJoint()
HAnimJoint59.DEF = "Joe_vt6"
HAnimJoint59.name = "vt6"
HAnimJoint59.center = [0,1.352,-0.12]
HAnimJoint60 = x3d.HAnimJoint()
HAnimJoint60.DEF = "Joe_vt5"
HAnimJoint60.name = "vt5"
HAnimJoint60.center = [0,1.381,-0.1235]
HAnimJoint61 = x3d.HAnimJoint()
HAnimJoint61.DEF = "Joe_vt4"
HAnimJoint61.name = "vt4"
HAnimJoint61.center = [0,1.41,-0.1235]
HAnimJoint61.skinCoordIndex = [81]
HAnimJoint61.skinCoordWeight = [1]
HAnimJoint62 = x3d.HAnimJoint()
HAnimJoint62.DEF = "Joe_vt3"
HAnimJoint62.name = "vt3"
HAnimJoint62.center = [0,1.438,-0.12]
HAnimJoint63 = x3d.HAnimJoint()
HAnimJoint63.DEF = "Joe_vt2"
HAnimJoint63.name = "vt2"
HAnimJoint63.center = [0,1.468,-0.105]
HAnimJoint64 = x3d.HAnimJoint()
HAnimJoint64.DEF = "Joe_vt1"
HAnimJoint64.name = "vt1"
HAnimJoint64.center = [0,1.497,-0.09]
HAnimJoint64.skinCoordIndex = [11,24]
HAnimJoint64.skinCoordWeight = [1,1]
HAnimJoint65 = x3d.HAnimJoint()
HAnimJoint65.DEF = "Joe_vc7"
HAnimJoint65.name = "vc7"
HAnimJoint65.center = [0,1.525,-0.072]
HAnimJoint65.skinCoordIndex = [74,75]
HAnimJoint65.skinCoordWeight = [1,1]
HAnimJoint66 = x3d.HAnimJoint()
HAnimJoint66.DEF = "Joe_vc6"
HAnimJoint66.name = "vc6"
HAnimJoint66.center = [0,1.54,-0.05]
HAnimJoint67 = x3d.HAnimJoint()
HAnimJoint67.DEF = "Joe_vc5"
HAnimJoint67.name = "vc5"
HAnimJoint67.center = [0,1.552,-0.035]
HAnimJoint68 = x3d.HAnimJoint()
HAnimJoint68.DEF = "Joe_vc4"
HAnimJoint68.name = "vc4"
HAnimJoint68.rotation = [-0.707106781186544,0,-0.707106781186544,0.184852582818168]
HAnimJoint68.center = [0,1.5675,-0.0256]
HAnimJoint69 = x3d.HAnimJoint()
HAnimJoint69.DEF = "Joe_vc3"
HAnimJoint69.name = "vc3"
HAnimJoint69.center = [0,1.58225,-0.0185]
HAnimJoint70 = x3d.HAnimJoint()
HAnimJoint70.DEF = "Joe_vc2"
HAnimJoint70.name = "vc2"
HAnimJoint70.center = [0,1.595,-0.0175]
HAnimJoint71 = x3d.HAnimJoint()
HAnimJoint71.DEF = "Joe_vc1"
HAnimJoint71.name = "vc1"
HAnimJoint71.center = [0,1.61,-0.015]
HAnimJoint72 = x3d.HAnimJoint()
HAnimJoint72.DEF = "Joe_skullbase"
HAnimJoint72.name = "skullbase"
HAnimJoint72.center = [0,1.63,-0.01]
HAnimJoint72.skinCoordIndex = [0,1,2,3,4,5,6,7,8,9]
HAnimJoint72.skinCoordWeight = [1,1,1,1,1,1,1,1,1,1]
HAnimDisplacer73 = x3d.HAnimDisplacer()
HAnimDisplacer73.DEF = "Joe_skull_tipTest"
HAnimDisplacer73.name = "skull_tip_raiser_action"
HAnimDisplacer73.coordIndex = [0,1,2,3,4,5,6,7,8,9]
HAnimDisplacer73.displacements = [(0, 0.15, 0),(0, 0, 0.15),(-0.1, 0, 0.15),(0.1, 0, 0.05),(0, -0.02, 0.05),(-0.15, 0, 0),(-0.05, 0, 0),(0.15, 0, 0),(0.05, 0, 0),(0, 0, -0.15)]

HAnimJoint72.displacers.append(HAnimDisplacer73)
HAnimJoint74 = x3d.HAnimJoint()
HAnimJoint74.DEF = "Joe_l_eyelid_joint"
HAnimJoint74.name = "l_eyelid_joint"
HAnimJoint74.center = [0.034,1.659,0.06]

HAnimJoint72.children.append(HAnimJoint74)
HAnimJoint75 = x3d.HAnimJoint()
HAnimJoint75.DEF = "Joe_l_eyeball_joint"
HAnimJoint75.name = "l_eyeball_joint"
HAnimJoint75.center = [0.034,1.659,0.06]

HAnimJoint72.children.append(HAnimJoint75)
HAnimJoint76 = x3d.HAnimJoint()
HAnimJoint76.DEF = "Joe_l_eyebrow_joint"
HAnimJoint76.name = "l_eyebrow_joint"
HAnimJoint76.center = [0.034,1.659,0.06]

HAnimJoint72.children.append(HAnimJoint76)
HAnimJoint77 = x3d.HAnimJoint()
HAnimJoint77.DEF = "Joe_r_eyelid_joint"
HAnimJoint77.name = "l_eyelid_joint"
HAnimJoint77.center = [-0.034,1.659,0.06]

HAnimJoint72.children.append(HAnimJoint77)
HAnimJoint78 = x3d.HAnimJoint()
HAnimJoint78.DEF = "Joe_r_eyeball_joint"
HAnimJoint78.name = "l_eyeball_joint"
HAnimJoint78.center = [-0.034,1.659,0.06]

HAnimJoint72.children.append(HAnimJoint78)
HAnimJoint79 = x3d.HAnimJoint()
HAnimJoint79.DEF = "Joe_r_eyebrow_joint"
HAnimJoint79.name = "l_eyebrow_joint"
HAnimJoint79.center = [-0.034,1.659,0.06]

HAnimJoint72.children.append(HAnimJoint79)
HAnimJoint80 = x3d.HAnimJoint()
HAnimJoint80.DEF = "Joe_temporomandibular"
HAnimJoint80.name = "temporomandibular"
HAnimJoint80.center = [0.034,1.659,0.06]

HAnimJoint72.children.append(HAnimJoint80)

HAnimJoint71.children.append(HAnimJoint72)

HAnimJoint70.children.append(HAnimJoint71)

HAnimJoint69.children.append(HAnimJoint70)

HAnimJoint68.children.append(HAnimJoint69)

HAnimJoint67.children.append(HAnimJoint68)

HAnimJoint66.children.append(HAnimJoint67)

HAnimJoint65.children.append(HAnimJoint66)
HAnimJoint81 = x3d.HAnimJoint()
HAnimJoint81.DEF = "Joe_l_sternoclavicular"
HAnimJoint81.name = "l_sternoclavicular"
HAnimJoint81.center = [0.082,1.4488,-0.0353]
HAnimJoint81.skinCoordIndex = [12]
HAnimJoint81.skinCoordWeight = [1]
HAnimJoint82 = x3d.HAnimJoint()
HAnimJoint82.DEF = "Joe_l_acromioclavicular"
HAnimJoint82.name = "l_acromioclavicular"
HAnimJoint82.center = [0.0962,1.4269,-0.0424]
HAnimJoint82.skinCoordIndex = [79]
HAnimJoint82.skinCoordWeight = [1]
HAnimJoint83 = x3d.HAnimJoint()
HAnimJoint83.DEF = "Joe_l_shoulder"
HAnimJoint83.name = "l_shoulder"
HAnimJoint83.rotation = [0,0,1,0.815348613033464]
HAnimJoint83.center = [0.2,1.44,-0.04]
HAnimJoint83.skinCoordIndex = [41,42,44,80,102,103,104,105]
HAnimJoint83.skinCoordWeight = [1,1,1,1,1,1,1,1]
HAnimJoint84 = x3d.HAnimJoint()
HAnimJoint84.DEF = "Joe_l_elbow"
HAnimJoint84.name = "l_elbow"
HAnimJoint84.rotation = [-1,0,0,2.02158164978027]
HAnimJoint84.center = [0.2,1.1388,-0.04]
HAnimJoint84.skinCoordIndex = [45,46,47,109,110,111,112,113,115,116,117,118]
HAnimJoint84.skinCoordWeight = [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]
HAnimJoint85 = x3d.HAnimJoint()
HAnimJoint85.DEF = "Joe_l_wrist"
HAnimJoint85.name = "l_wrist"
HAnimJoint85.rotation = [0,1,0,0.706635464629002]
HAnimJoint85.center = [0.2,0.87,-0.04]
HAnimJoint85.skinCoordIndex = [119,120,121,122,123,124,125,126]
HAnimJoint85.skinCoordWeight = [1,1,1,1,1,1,1,1]
HAnimJoint86 = x3d.HAnimJoint()
HAnimJoint86.DEF = "Joe_l_thumb1"
HAnimJoint86.name = "l_thumb1"
HAnimJoint86.center = [0.1924,0.8472,-0.0534]
HAnimJoint86.skinCoordIndex = [127,128]
HAnimJoint86.skinCoordWeight = [1,1]
HAnimJoint87 = x3d.HAnimJoint()
HAnimJoint87.DEF = "Joe_l_thumb2"
HAnimJoint87.name = "l_thumb2"
HAnimJoint87.center = [0.1951,0.8226,0.0246]
HAnimJoint87.skinCoordIndex = [138,139,140,141,142,143]
HAnimJoint87.skinCoordWeight = [0.5,0.5,0.5,1,1,1]
HAnimJoint88 = x3d.HAnimJoint()
HAnimJoint88.DEF = "Joe_l_thumb3"
HAnimJoint88.name = "l_thumb3"
HAnimJoint88.center = [0.1955,0.8159,0.0464]
HAnimJoint88.skinCoordIndex = [144,145,146,147,148,149,150,151,152]
HAnimJoint88.skinCoordWeight = [1,1,1,1,1,1,1,1,1]

HAnimJoint87.children.append(HAnimJoint88)

HAnimJoint86.children.append(HAnimJoint87)

HAnimJoint85.children.append(HAnimJoint86)
HAnimJoint89 = x3d.HAnimJoint()
HAnimJoint89.DEF = "Joe_l_index0"
HAnimJoint89.name = "l_index0"
HAnimJoint89.center = [0.1983,0.8024,-0.028]
HAnimJoint89.skinCoordIndex = [129,130]
HAnimJoint89.skinCoordWeight = [1,1]
HAnimJoint90 = x3d.HAnimJoint()
HAnimJoint90.DEF = "Joe_l_index1"
HAnimJoint90.name = "l_index1"
HAnimJoint90.center = [0.1983,0.7815,-0.028]
HAnimJoint90.skinCoordIndex = [138,139,140,153,154,155,163]
HAnimJoint90.skinCoordWeight = [0.5,0.5,0.5,1,1,1,0.5]
HAnimJoint91 = x3d.HAnimJoint()
HAnimJoint91.DEF = "Joe_l_index2"
HAnimJoint91.name = "l_index2"
HAnimJoint91.center = [0.2017,0.7363,-0.0248]
HAnimJoint91.skinCoordIndex = [166,167,168,169]
HAnimJoint91.skinCoordWeight = [1,1,1,1]
HAnimJoint92 = x3d.HAnimJoint()
HAnimJoint92.DEF = "Joe_l_index3"
HAnimJoint92.name = "l_index3"
HAnimJoint92.center = [0.2028,0.7139,-0.0236]
HAnimJoint92.skinCoordIndex = [170,171,172,173,174,175,176,177,178]
HAnimJoint92.skinCoordWeight = [1,1,1,1,1,1,1,1,1]

HAnimJoint91.children.append(HAnimJoint92)

HAnimJoint90.children.append(HAnimJoint91)

HAnimJoint89.children.append(HAnimJoint90)

HAnimJoint85.children.append(HAnimJoint89)
HAnimJoint93 = x3d.HAnimJoint()
HAnimJoint93.DEF = "Joe_l_middle0"
HAnimJoint93.name = "l_middle0"
HAnimJoint93.center = [0.1987,0.8029,-0.053]
HAnimJoint93.skinCoordIndex = [131,132]
HAnimJoint93.skinCoordWeight = [1,1]
HAnimJoint94 = x3d.HAnimJoint()
HAnimJoint94.DEF = "Joe_l_middle1"
HAnimJoint94.name = "l_middle1"
HAnimJoint94.center = [0.1987,0.7818,-0.053]
HAnimJoint94.skinCoordIndex = [156,157,163,164]
HAnimJoint94.skinCoordWeight = [1,1,0.5,0.5]
HAnimJoint95 = x3d.HAnimJoint()
HAnimJoint95.DEF = "Joe_l_middle2"
HAnimJoint95.name = "l_middle2"
HAnimJoint95.center = [0.2013,0.7273,-0.0503]
HAnimJoint95.skinCoordIndex = [179,180,181,182]
HAnimJoint95.skinCoordWeight = [1,1,1,1]
HAnimJoint96 = x3d.HAnimJoint()
HAnimJoint96.DEF = "Joe_l_middle3"
HAnimJoint96.name = "l_middle3"
HAnimJoint96.center = [0.2026,0.7011,-0.0494]
HAnimJoint96.skinCoordIndex = [183,184,185,186,187,188,189,190,191]
HAnimJoint96.skinCoordWeight = [1,1,1,1,1,1,1,1,1]

HAnimJoint95.children.append(HAnimJoint96)

HAnimJoint94.children.append(HAnimJoint95)

HAnimJoint93.children.append(HAnimJoint94)

HAnimJoint85.children.append(HAnimJoint93)
HAnimJoint97 = x3d.HAnimJoint()
HAnimJoint97.DEF = "Joe_l_ring0"
HAnimJoint97.name = "l_ring0"
HAnimJoint97.center = [0.1956,0.8019,-0.0794]
HAnimJoint97.skinCoordIndex = [133,134]
HAnimJoint97.skinCoordWeight = [1,1]
HAnimJoint98 = x3d.HAnimJoint()
HAnimJoint98.DEF = "Joe_l_ring1"
HAnimJoint98.name = "l_ring1"
HAnimJoint98.center = [0.1956,0.7815,-0.0794]
HAnimJoint98.skinCoordIndex = [158,159,164,165]
HAnimJoint98.skinCoordWeight = [1,1,0.5,0.5]
HAnimJoint99 = x3d.HAnimJoint()
HAnimJoint99.DEF = "Joe_l_ring2"
HAnimJoint99.name = "l_ring2"
HAnimJoint99.center = [0.1973,0.7287,-0.0777]
HAnimJoint99.skinCoordIndex = [192,193,194,195]
HAnimJoint99.skinCoordWeight = [1,1,1,1]
HAnimJoint100 = x3d.HAnimJoint()
HAnimJoint100.DEF = "Joe_l_ring3"
HAnimJoint100.name = "l_ring3"
HAnimJoint100.center = [0.1983,0.7045,-0.0767]
HAnimJoint100.skinCoordIndex = [196,197,198,199,200,201,202,203,204]
HAnimJoint100.skinCoordWeight = [1,1,1,1,1,1,1,1,1]

HAnimJoint99.children.append(HAnimJoint100)

HAnimJoint98.children.append(HAnimJoint99)

HAnimJoint97.children.append(HAnimJoint98)

HAnimJoint85.children.append(HAnimJoint97)
HAnimJoint101 = x3d.HAnimJoint()
HAnimJoint101.DEF = "Joe_l_pinky0"
HAnimJoint101.name = "l_pinky0"
HAnimJoint101.center = [0.1925,0.8066,-0.1036]
HAnimJoint101.skinCoordIndex = [135,136,137,165]
HAnimJoint101.skinCoordWeight = [1,1,1,0.5]
HAnimJoint102 = x3d.HAnimJoint()
HAnimJoint102.DEF = "Joe_l_pinky1"
HAnimJoint102.name = "l_pinky1"
HAnimJoint102.center = [0.1925,0.7866,-0.1036]
HAnimJoint102.skinCoordIndex = [160,161,162]
HAnimJoint102.skinCoordWeight = [1,1,1]
HAnimJoint103 = x3d.HAnimJoint()
HAnimJoint103.DEF = "Joe_l_pinky2"
HAnimJoint103.name = "l_pinky2"
HAnimJoint103.center = [0.1938,0.7452,-0.1024]
HAnimJoint103.skinCoordIndex = [205,206,207,208]
HAnimJoint103.skinCoordWeight = [1,1,1,1]
HAnimJoint104 = x3d.HAnimJoint()
HAnimJoint104.DEF = "Joe_l_pinky3"
HAnimJoint104.name = "l_pinky3"
HAnimJoint104.center = [0.1948,0.7277,-0.1017]
HAnimJoint104.skinCoordIndex = [209,210,211,212,213,214,215,216,217]
HAnimJoint104.skinCoordWeight = [1,1,1,1,1,1,1,1,1]

HAnimJoint103.children.append(HAnimJoint104)

HAnimJoint102.children.append(HAnimJoint103)

HAnimJoint101.children.append(HAnimJoint102)

HAnimJoint85.children.append(HAnimJoint101)

HAnimJoint84.children.append(HAnimJoint85)

HAnimJoint83.children.append(HAnimJoint84)

HAnimJoint82.children.append(HAnimJoint83)

HAnimJoint81.children.append(HAnimJoint82)

HAnimJoint65.children.append(HAnimJoint81)
HAnimJoint105 = x3d.HAnimJoint()
HAnimJoint105.DEF = "Joe_r_sternoclavicular"
HAnimJoint105.name = "r_sternoclavicular"
HAnimJoint105.center = [-0.03,1.46,0]
HAnimJoint105.skinCoordIndex = [10]
HAnimJoint105.skinCoordWeight = [1]
HAnimJoint106 = x3d.HAnimJoint()
HAnimJoint106.DEF = "Joe_r_acromioclavicular"
HAnimJoint106.name = "r_acromioclavicular"
HAnimJoint106.center = [-0.09,1.41,-0.11]
HAnimJoint106.skinCoordIndex = [77,29]
HAnimJoint106.skinCoordWeight = [1,0.9]
HAnimJoint107 = x3d.HAnimJoint()
HAnimJoint107.DEF = "Joe_r_shoulder"
HAnimJoint107.name = "r_shoulder"
HAnimJoint107.rotation = [0,0,-1,2.06514739990234]
HAnimJoint107.center = [-0.2,1.44,-0.04]
HAnimJoint107.skinCoordIndex = [29,30,32,78,218,219,220,221,86,88]
HAnimJoint107.skinCoordWeight = [0.1,1,1,1,1,1,1,1,0.3,0.2]
HAnimJoint108 = x3d.HAnimJoint()
HAnimJoint108.DEF = "Joe_r_elbow"
HAnimJoint108.name = "r_elbow"
HAnimJoint108.rotation = [-1,0,0,2.02158164978027]
HAnimJoint108.center = [-0.2,1.1388,-0.04]
HAnimJoint108.skinCoordIndex = [33,34,35,225,226,227,228,229,231,232,233,234]
HAnimJoint108.skinCoordWeight = [1,1,1,1,1,1,1,1,1,1,1,1]
HAnimJoint109 = x3d.HAnimJoint()
HAnimJoint109.DEF = "Joe_r_wrist"
HAnimJoint109.name = "r_wrist"
HAnimJoint109.rotation = [0,0.999999999999986,0,0.169544219970706]
HAnimJoint109.center = [-0.2,0.89,-0.04]
HAnimJoint109.skinCoordIndex = [235,236,237,238,239,240,241,242]
HAnimJoint109.skinCoordWeight = [1,1,1,1,1,1,1,1]
HAnimJoint110 = x3d.HAnimJoint()
HAnimJoint110.DEF = "Joe_r_thumb1"
HAnimJoint110.name = "r_thumb1"
HAnimJoint110.center = [-0.2,0.85,0]
HAnimJoint110.skinCoordIndex = [243,244]
HAnimJoint110.skinCoordWeight = [1,1]
HAnimJoint111 = x3d.HAnimJoint()
HAnimJoint111.DEF = "Joe_r_thumb2"
HAnimJoint111.name = "r_thumb2"
HAnimJoint111.center = [-0.2,0.82,0.03]
HAnimJoint111.skinCoordIndex = [254,255,256,257,258,259]
HAnimJoint111.skinCoordWeight = [0.5,0.5,0.5,1,1,1]
HAnimJoint112 = x3d.HAnimJoint()
HAnimJoint112.DEF = "Joe_r_thumb3"
HAnimJoint112.name = "r_thumb3"
HAnimJoint112.center = [-0.2,0.8,0.05]
HAnimJoint112.skinCoordIndex = [260,261,262,263,264,265,266,267,268]
HAnimJoint112.skinCoordWeight = [1,1,1,1,1,1,1,1,1]

HAnimJoint111.children.append(HAnimJoint112)

HAnimJoint110.children.append(HAnimJoint111)

HAnimJoint109.children.append(HAnimJoint110)
HAnimJoint113 = x3d.HAnimJoint()
HAnimJoint113.DEF = "Joe_r_index0"
HAnimJoint113.name = "r_index0"
HAnimJoint113.center = [-0.2,0.84,-0.015]
HAnimJoint113.skinCoordIndex = [245,246]
HAnimJoint113.skinCoordWeight = [1,1]
HAnimJoint114 = x3d.HAnimJoint()
HAnimJoint114.DEF = "Joe_r_index1"
HAnimJoint114.name = "r_index1"
HAnimJoint114.center = [-0.2,0.793,-0.015]
HAnimJoint114.skinCoordIndex = [254,255,256,269,270,271,279]
HAnimJoint114.skinCoordWeight = [0.5,0.5,0.5,1,1,1,0.5]
HAnimJoint115 = x3d.HAnimJoint()
HAnimJoint115.DEF = "Joe_r_index2"
HAnimJoint115.name = "r_index2"
HAnimJoint115.center = [-0.2,0.745,-0.015]
HAnimJoint115.skinCoordIndex = [282,283,284,285]
HAnimJoint115.skinCoordWeight = [1,1,1,1]
HAnimJoint116 = x3d.HAnimJoint()
HAnimJoint116.DEF = "Joe_r_index3"
HAnimJoint116.name = "r_index3"
HAnimJoint116.center = [-0.2,0.72,-0.015]
HAnimJoint116.skinCoordIndex = [286,287,288,289,290,291,292,293,294]
HAnimJoint116.skinCoordWeight = [1,1,1,1,1,1,1,1,1]

HAnimJoint115.children.append(HAnimJoint116)

HAnimJoint114.children.append(HAnimJoint115)

HAnimJoint113.children.append(HAnimJoint114)

HAnimJoint109.children.append(HAnimJoint113)
HAnimJoint117 = x3d.HAnimJoint()
HAnimJoint117.DEF = "Joe_r_middle0"
HAnimJoint117.name = "r_middle0"
HAnimJoint117.center = [-0.2,0.835,-0.04]
HAnimJoint117.skinCoordIndex = [247,248]
HAnimJoint117.skinCoordWeight = [1,1]
HAnimJoint118 = x3d.HAnimJoint()
HAnimJoint118.DEF = "Joe_r_middle1"
HAnimJoint118.name = "r_middle1"
HAnimJoint118.center = [-0.2,0.788,-0.04]
HAnimJoint118.skinCoordIndex = [272,273,279,280]
HAnimJoint118.skinCoordWeight = [1,1,0.5,0.5]
HAnimJoint119 = x3d.HAnimJoint()
HAnimJoint119.DEF = "Joe_r_middle2"
HAnimJoint119.name = "r_middle2"
HAnimJoint119.center = [-0.2,0.74,-0.04]
HAnimJoint119.skinCoordIndex = [295,296,297,298]
HAnimJoint119.skinCoordWeight = [1,1,1,1]
HAnimJoint120 = x3d.HAnimJoint()
HAnimJoint120.DEF = "Joe_r_middle3"
HAnimJoint120.name = "r_middle3"
HAnimJoint120.center = [-0.2,0.7142,-0.04]
HAnimJoint120.skinCoordIndex = [299,300,301,302,303,304,305,306,307]
HAnimJoint120.skinCoordWeight = [1,1,1,1,1,1,1,1,1]

HAnimJoint119.children.append(HAnimJoint120)

HAnimJoint118.children.append(HAnimJoint119)

HAnimJoint117.children.append(HAnimJoint118)

HAnimJoint109.children.append(HAnimJoint117)
HAnimJoint121 = x3d.HAnimJoint()
HAnimJoint121.DEF = "Joe_r_ring0"
HAnimJoint121.name = "r_ring0"
HAnimJoint121.center = [-0.2,0.835,-0.065]
HAnimJoint121.skinCoordIndex = [249,250]
HAnimJoint121.skinCoordWeight = [1,1]
HAnimJoint122 = x3d.HAnimJoint()
HAnimJoint122.DEF = "Joe_r_ring1"
HAnimJoint122.name = "r_ring1"
HAnimJoint122.center = [-0.2,0.793,-0.065]
HAnimJoint122.skinCoordIndex = [274,275,280,281]
HAnimJoint122.skinCoordWeight = [1,1,0.5,0.5]
HAnimJoint123 = x3d.HAnimJoint()
HAnimJoint123.DEF = "Joe_r_ring2"
HAnimJoint123.name = "r_ring2"
HAnimJoint123.center = [-0.2,0.74,-0.065]
HAnimJoint123.skinCoordIndex = [308,309,310,311]
HAnimJoint123.skinCoordWeight = [1,1,1,1]
HAnimJoint124 = x3d.HAnimJoint()
HAnimJoint124.DEF = "Joe_r_ring3"
HAnimJoint124.name = "r_ring3"
HAnimJoint124.center = [-0.2,0.7177,-0.065]
HAnimJoint124.skinCoordIndex = [312,313,314,315,316,317,318,319,320]
HAnimJoint124.skinCoordWeight = [1,1,1,1,1,1,1,1,1]

HAnimJoint123.children.append(HAnimJoint124)

HAnimJoint122.children.append(HAnimJoint123)

HAnimJoint121.children.append(HAnimJoint122)

HAnimJoint109.children.append(HAnimJoint121)
HAnimJoint125 = x3d.HAnimJoint()
HAnimJoint125.DEF = "Joe_r_pinky0"
HAnimJoint125.name = "r_pinky0"
HAnimJoint125.center = [-0.2,0.84,-0.085]
HAnimJoint125.skinCoordIndex = [251,252,253,281]
HAnimJoint125.skinCoordWeight = [1,1,1,0.5]
HAnimJoint126 = x3d.HAnimJoint()
HAnimJoint126.DEF = "Joe_r_pinky1"
HAnimJoint126.name = "r_pinky1"
HAnimJoint126.center = [-0.2,0.79,-0.085]
HAnimJoint126.skinCoordIndex = [276,277,278]
HAnimJoint126.skinCoordWeight = [1,1,1]
HAnimJoint127 = x3d.HAnimJoint()
HAnimJoint127.DEF = "Joe_r_pinky2"
HAnimJoint127.name = "r_pinky2"
HAnimJoint127.center = [-0.2,0.755,-0.085]
HAnimJoint127.skinCoordIndex = [321,322,323,324]
HAnimJoint127.skinCoordWeight = [1,1,1,1]
HAnimJoint128 = x3d.HAnimJoint()
HAnimJoint128.DEF = "Joe_r_pinky3"
HAnimJoint128.name = "r_pinky3"
HAnimJoint128.center = [-0.2,0.735,-0.09]
HAnimJoint128.skinCoordIndex = [325,326,327,328,329,330,331,332,333]
HAnimJoint128.skinCoordWeight = [1,1,1,1,1,1,1,1,1]

HAnimJoint127.children.append(HAnimJoint128)

HAnimJoint126.children.append(HAnimJoint127)

HAnimJoint125.children.append(HAnimJoint126)

HAnimJoint109.children.append(HAnimJoint125)

HAnimJoint108.children.append(HAnimJoint109)

HAnimJoint107.children.append(HAnimJoint108)

HAnimJoint106.children.append(HAnimJoint107)

HAnimJoint105.children.append(HAnimJoint106)

HAnimJoint65.children.append(HAnimJoint105)

HAnimJoint64.children.append(HAnimJoint65)

HAnimJoint63.children.append(HAnimJoint64)

HAnimJoint62.children.append(HAnimJoint63)

HAnimJoint61.children.append(HAnimJoint62)

HAnimJoint60.children.append(HAnimJoint61)

HAnimJoint59.children.append(HAnimJoint60)

HAnimJoint58.children.append(HAnimJoint59)

HAnimJoint57.children.append(HAnimJoint58)

HAnimJoint56.children.append(HAnimJoint57)

HAnimJoint55.children.append(HAnimJoint56)

HAnimJoint54.children.append(HAnimJoint55)

HAnimJoint53.children.append(HAnimJoint54)

HAnimJoint52.children.append(HAnimJoint53)

HAnimJoint51.children.append(HAnimJoint52)

HAnimJoint50.children.append(HAnimJoint51)

HAnimJoint49.children.append(HAnimJoint50)

HAnimJoint48.children.append(HAnimJoint49)

HAnimJoint26.children.append(HAnimJoint48)

HAnimHumanoid25.skeleton.append(HAnimJoint26)
Coordinate129 = x3d.Coordinate()
Coordinate129.DEF = "Joe_SkinCoord"
Coordinate129.point = [(0, 1.77, 0),(0, 1.665, 0.09),(-0.033, 1.62, 0.087),(0.033, 1.62, 0.087),(0, 1.55, 0.097),(-0.077, 1.64, -0.01),(-0.0527, 1.58, 0.015),(0.077, 1.64, -0.01),(0.0527, 1.58, 0.015),(0, 1.625, -0.0925),(-0.03, 1.46, 0.035),(0, 1.44, 0.03),(0.03, 1.46, 0.035),(-0.1135, 1.318, 0.095),(0.1135, 1.318, 0.095),(0, 1.25, 0.113),(-0.087, 1.19, 0.09),(-0.0935, 1.03, 0.075),(0.087, 1.19, 0.09),(0.0935, 1.03, 0.075),(-0.1425, 1.065, 0.0033),(-0.15, 0.9, -0.01),(0.1425, 1.065, 0.0033),(0.15, 0.9, -0.01),(0, 1.53, -0.084),(0.0049, 1.1908, -0.1113),(-0.0773, 1.019, -0.12),(0.0773, 1.019, -0.12),(0.005, 1.0915, -0.1091),(-0.178, 1.4825, -0.0625),(-0.17, 1.38, 0.007),(-0.1884, 0.8676, -0.036),(-0.16, 1.38, -0.127),(-0.2, 1.1388, -0.08),(-0.244, 1.1388, -0.04),(-0.165, 1.1388, -0.04),(-0.23, 1.133, -0.055),(-0.1977, 0.8169, -0.0177),(-0.1941, 0.6772, -0.0423),(-0.2117, 0.8562, -0.0584),(-0.1929, 0.789, -0.1064),(0.175, 1.4825, -0.06),(0.17, 1.38, 0.007),(0.1901, 0.8645, -0.0415),(0.16, 1.38, -0.125),(0.2, 1.1388, -0.08),(0.165, 1.1388, -0.04),(0.244, 1.1388, -0.04),(0.23, 1.133, -0.055),(0.2009, 0.8139, -0.0237),(0.2056, 0.6743, -0.0482),(0.2142, 0.8529, -0.0648),(0.1929, 0.786, -0.1122),(-0.1, 0.4913, -0.03),(-0.17, 0.466, 0),(-0.05, 0.466, 0),(-0.165, 0.01, 0.12),(-0.15, 0.07, 0),(-0.085, 0.086, 0.0125),(-0.09, 0.056, 0.0125),(-0.115, 0.02, 0.122),(-0.115, 0.04, -0.055),(-0.11, 0.011, 0.19),(0.0993, 0.4881, -0.0309),(0.17, 0.466, 0),(0.05, 0.4867, 0),(0.165, 0.01, 0.12),(0.15, 0.07, 0),(0.085, 0.086, 0.0125),(0.09, 0.056, 0.0125),(0.115, 0.02, 0.122),(0.115, 0.04, -0.055),(0.11, 0.011, 0.19),(0, 0.875, 0),(-0.0646, 1.5149, -0.038),(0.0646, 1.5149, -0.038),(0, 1.07225, 0.09),(-0.11, 1.427, -0.1375),(-0.235, 1.42, -0.0625),(0.11, 1.427, -0.1375),(0.235, 1.42, -0.0625),(0, 1.41, -0.145),(0, 0.925, 0.08),(-0.087, 1.19, -0.09),(0.087, 1.19, -0.09),(0.172, 1.32, -0.03),(-0.172, 1.32, -0.03),(0.15, 1.23, -0.015),(-0.15, 1.23, -0.015),(0.079, 0.92, -0.14),(0.1, 0.9, 0.077),(-0.079, 0.92, -0.14),(-0.1, 0.9, 0.075),(0, 0.87, 0),(0.171, 0.65, 0),(0.02, 0.65, 0),(0.1, 0.65, -0.08),(0.1, 0.65, 0.07),(-0.171, 0.65, 0),(-0.02, 0.65, 0),(-0.1, 0.65, -0.08),(-0.1, 0.65, 0.07),(0.25, 1.27, -0.04),(0.17, 1.27, -0.04),(0.2, 1.27, -0.09),(0.2, 1.27, 0.02),(0.244, 1.1388, -0.04),(0.165, 1.1388, -0.04),(0.2, 1.1388, -0.08),(0.2, 1.1388, -0.013),(0.225, 1, -0.01),(0.225, 1, -0.07),(0.185, 1, -0.01),(0.185, 1, -0.07),(0.2, 1.1388, -0.04),(0.225, 0.92, -0.04),(0.175, 0.92, -0.04),(0.2, 0.92, -0.065),(0.2, 0.92, -0.015),(0.225, 0.89, -0.04),(0.175, 0.89, -0.04),(0.2, 0.89, -0.065),(0.2, 0.89, -0.015),(0.218, 0.86, -0.04),(0.184, 0.86, -0.04),(0.2, 0.87, -0.07),(0.2, 0.87, 0),(0.21, 0.85, 0),(0.1854, 0.85, 0),(0.212, 0.84, -0.015),(0.183, 0.84, -0.015),(0.213, 0.835, -0.04),(0.19, 0.835, -0.04),(0.211, 0.835, -0.065),(0.192, 0.835, -0.065),(0.208, 0.84, -0.085),(0.19, 0.84, -0.085),(0.2, 0.84, -0.095),(0.215, 0.82, 0),(0.193, 0.815, 0.005),(0.198, 0.8, 0.012),(0.21, 0.82, 0.03),(0.19, 0.82, 0.03),(0.2, 0.835, 0.039),(0.212, 0.8, 0.05),(0.188, 0.8, 0.05),(0.2, 0.807, 0.057),(0.2, 0.793, 0.035),(0.2, 0.774, 0.076),(0.212, 0.78, 0.07),(0.188, 0.78, 0.07),(0.2, 0.785, 0.075),(0.2, 0.77, 0.062),(0.215, 0.793, -0.015),(0.187, 0.793, -0.015),(0.2, 0.793, -0.005),(0.215, 0.788, -0.04),(0.187, 0.788, -0.04),(0.215, 0.793, -0.065),(0.187, 0.793, -0.065),(0.21, 0.79, -0.085),(0.19, 0.79, -0.085),(0.2, 0.79, -0.095),(0.19, 0.77, -0.0275),(0.19, 0.77, -0.0525),(0.19, 0.78, -0.0775),(0.212, 0.745, -0.015),(0.188, 0.745, -0.02),(0.2, 0.745, -0.0255),(0.2, 0.745, -0.0045),(0.211, 0.72, -0.015),(0.189, 0.72, -0.015),(0.2, 0.72, -0.0252),(0.2, 0.72, -0.0048),(0.21, 0.695, -0.015),(0.19, 0.695, -0.015),(0.2, 0.695, -0.025),(0.2, 0.695, -0.005),(0.2, 0.685, -0.015),(0.215, 0.74, -0.04),(0.185, 0.74, -0.04),(0.2, 0.74, -0.055),(0.2, 0.74, -0.025),(0.21, 0.7142, -0.04),(0.19, 0.7142, -0.04),(0.2, 0.7142, -0.053),(0.2, 0.7142, -0.027),(0.21, 0.68, -0.04),(0.19, 0.68, -0.04),(0.2, 0.68, -0.05),(0.2, 0.68, -0.03),(0.2, 0.67, -0.04),(0.212, 0.74, -0.065),(0.188, 0.74, -0.065),(0.2, 0.74, -0.0756),(0.2, 0.74, -0.0542),(0.21, 0.7177, -0.065),(0.19, 0.7177, -0.065),(0.2, 0.7177, -0.0751),(0.2, 0.7177, -0.0549),(0.21, 0.695, -0.065),(0.19, 0.695, -0.065),(0.2, 0.695, -0.075),(0.2, 0.695, -0.055),(0.2, 0.685, -0.065),(0.211, 0.755, -0.085),(0.189, 0.755, -0.085),(0.2, 0.755, -0.0952),(0.2, 0.755, -0.0748),(0.21, 0.735, -0.085),(0.19, 0.735, -0.085),(0.2, 0.735, -0.0951),(0.2, 0.735, -0.0749),(0.21, 0.72, -0.085),(0.19, 0.72, -0.085),(0.2, 0.72, -0.095),(0.2, 0.72, -0.075),(0.2, 0.71, -0.085),(-0.23, 1.23, -0.04),(-0.16, 1.23, -0.04),(-0.2, 1.235, -0.105),(-0.2, 1.255, 0.02),(-0.244, 1.1388, -0.04),(-0.165, 1.1388, -0.04),(-0.2, 1.1388, -0.08),(-0.2, 1.1388, 0.013),(-0.225, 1, -0.01),(-0.225, 1, -0.07),(-0.185, 1, -0.01),(-0.185, 1, -0.07),(-0.2, 1.1388, -0.04),(-0.225, 0.92, -0.04),(-0.175, 0.92, -0.04),(-0.2, 0.92, -0.065),(-0.2, 0.92, -0.015),(-0.225, 0.89, -0.04),(-0.175, 0.89, -0.04),(-0.2, 0.89, -0.065),(-0.2, 0.89, -0.015),(-0.218, 0.86, -0.04),(-0.184, 0.86, -0.04),(-0.2, 0.87, -0.07),(-0.2, 0.87, 0),(-0.21, 0.85, 0),(-0.1854, 0.85, 0),(-0.212, 0.84, -0.015),(-0.183, 0.84, -0.015),(-0.213, 0.835, -0.04),(-0.19, 0.835, -0.04),(-0.211, 0.835, -0.065),(-0.192, 0.835, -0.065),(-0.208, 0.84, -0.085),(-0.19, 0.84, -0.085),(-0.2, 0.84, -0.095),(-0.215, 0.82, 0),(-0.193, 0.815, 0.005),(-0.198, 0.8, 0.012),(-0.21, 0.82, 0.03),(-0.19, 0.82, 0.03),(-0.2, 0.835, 0.039),(-0.212, 0.8, 0.05),(-0.188, 0.8, 0.05),(-0.2, 0.807, 0.057),(-0.2, 0.793, 0.035),(-0.2, 0.774, 0.076),(-0.212, 0.78, 0.07),(-0.188, 0.78, 0.07),(-0.2, 0.785, 0.075),(-0.2, 0.77, 0.062),(-0.215, 0.793, -0.015),(-0.187, 0.793, -0.015),(-0.2, 0.793, -0.005),(-0.215, 0.788, -0.04),(-0.187, 0.788, -0.04),(-0.215, 0.793, -0.065),(-0.187, 0.793, -0.065),(-0.21, 0.79, -0.085),(-0.19, 0.79, -0.085),(-0.2, 0.79, -0.095),(-0.19, 0.77, -0.0275),(-0.19, 0.77, -0.0525),(-0.19, 0.78, -0.0775),(-0.212, 0.745, -0.015),(-0.188, 0.745, -0.02),(-0.2, 0.745, -0.0255),(-0.2, 0.745, -0.0045),(-0.211, 0.72, -0.015),(-0.189, 0.72, -0.015),(-0.2, 0.72, -0.0252),(-0.2, 0.72, -0.0048),(-0.21, 0.695, -0.015),(-0.19, 0.695, -0.015),(-0.2, 0.695, -0.025),(-0.2, 0.695, -0.005),(-0.2, 0.685, -0.015),(-0.215, 0.74, -0.04),(-0.185, 0.74, -0.04),(-0.2, 0.74, -0.055),(-0.2, 0.74, -0.025),(-0.21, 0.7142, -0.04),(-0.19, 0.7142, -0.04),(-0.2, 0.7142, -0.053),(-0.2, 0.7142, -0.027),(-0.21, 0.68, -0.04),(-0.19, 0.68, -0.04),(-0.2, 0.68, -0.05),(-0.2, 0.68, -0.03),(-0.2, 0.67, -0.04),(-0.212, 0.74, -0.065),(-0.188, 0.74, -0.065),(-0.2, 0.74, -0.0756),(-0.2, 0.74, -0.0542),(-0.21, 0.7177, -0.065),(-0.19, 0.7177, -0.065),(-0.2, 0.7177, -0.0751),(-0.2, 0.7177, -0.0549),(-0.21, 0.695, -0.065),(-0.19, 0.695, -0.065),(-0.2, 0.695, -0.075),(-0.2, 0.695, -0.055),(-0.2, 0.685, -0.065),(-0.211, 0.755, -0.085),(-0.189, 0.755, -0.085),(-0.2, 0.755, -0.0952),(-0.2, 0.755, -0.0748),(-0.21, 0.735, -0.085),(-0.19, 0.735, -0.085),(-0.2, 0.735, -0.0951),(-0.2, 0.735, -0.0749),(-0.21, 0.72, -0.085),(-0.19, 0.72, -0.085),(-0.2, 0.72, -0.095),(-0.2, 0.72, -0.075),(-0.2, 0.71, -0.085),(0.115, 0.466, 0.06),(0.115, 0.466, -0.055),(0.15, 0.466, 0),(0.05, 0.466, 0),(0.17, 0.3, 0),(0.06, 0.3, 0),(0.1, 0.3, -0.05),(0.1, 0.3, 0.05),(0.15, 0.07, 0),(0.085, 0.086, 0.0125),(0.115, 0.069, -0.045),(0.117, 0.0975, 0.0615),(0.1375, 0.006, -0.03),(0.095, 0.006, -0.03),(0.115, 0.015, -0.045),(0.115, 0.06, 0.1),(0.115, 0, 0.07),(0.165, 0, 0.07),(0.095, 0, 0.07),(0.115, 0.04, 0.13),(0.125, 0, 0.12),(0.165, 0, 0.12),(0.087, 0, 0.122),(0.09, 0.012, 0.188),(0.11, 0.011, 0.19),(0.128, 0.011, 0.185),(0.142, 0.011, 0.178),(0.154, 0.01, 0.168),(-0.115, 0.466, 0.06),(-0.115, 0.466, -0.055),(-0.17, 0.466, 0),(-0.05, 0.466, 0),(-0.17, 0.3, 0),(-0.06, 0.3, 0),(-0.1, 0.3, -0.05),(-0.1, 0.3, 0.05),(-0.15, 0.07, 0),(-0.085, 0.086, 0.0125),(-0.115, 0.069, -0.045),(-0.117, 0.0975, 0.0615),(-0.1375, 0.006, -0.03),(-0.095, 0.006, -0.03),(-0.095, 0.006, -0.03),(-0.115, 0.06, 0.1),(-0.115, 0, 0.07),(-0.165, 0, 0.07),(-0.095, 0, 0.07),(-0.115, 0.04, 0.13),(-0.125, 0, 0.12),(-0.165, 0, 0.12),(-0.087, 0, 0.122),(-0.09, 0.012, 0.188),(-0.11, 0.011, 0.19),(-0.128, 0.011, 0.185),(-0.142, 0.011, 0.178),(-0.154, 0.01, 0.168)]

HAnimHumanoid25.skinCoord = Coordinate129
Shape130 = x3d.Shape()
Shape130.DEF = "Joe_Shape"
Appearance131 = x3d.Appearance()
Appearance131.DEF = "Joe_skin_Appearance"
Material132 = x3d.Material()
Material132.DEF = "Joe_skin_Material"
Material132.diffuseColor = [0.3,0.3,0.6]
Material132.emissiveColor = [0.3,0.3,0.6]

Appearance131.material = Material132
ImageTexture133 = x3d.ImageTexture()
ImageTexture133.DEF = "JoeSkinImageTexture"
ImageTexture133.url = ["bodytexture28.png"]

Appearance131.texture = ImageTexture133
TextureTransform134 = x3d.TextureTransform()
TextureTransform134.DEF = "kicktextrans"

Appearance131.textureTransform = TextureTransform134

Shape130.appearance = Appearance131
IndexedFaceSet135 = x3d.IndexedFaceSet()
IndexedFaceSet135.DEF = "Joe_skin_IndexedFaceSet"
IndexedFaceSet135.creaseAngle = 3.14
IndexedFaceSet135.coordIndex = [0,9,5,-1,0,7,9,-1,0,5,1,-1,1,5,2,-1,1,3,7,-1,2,4,3,-1,0,1,7,-1,1,2,3,-1,5,6,2,-1,7,3,8,-1,6,4,2,-1,3,4,8,-1,9,6,5,-1,9,7,8,-1,4,6,10,-1,4,10,12,-1,4,12,8,-1,10,11,12,-1,9,75,24,-1,9,24,74,-1,9,8,75,-1,9,74,6,-1,10,6,74,-1,12,75,8,-1,74,24,29,-1,24,77,29,-1,10,74,29,-1,77,32,29,-1,32,78,29,-1,78,30,29,-1,30,10,29,-1,41,24,75,-1,41,75,12,-1,41,12,42,-1,41,42,80,-1,41,80,44,-1,41,44,79,-1,41,79,24,-1,81,24,79,-1,81,77,24,-1,81,25,77,-1,81,79,25,-1,25,79,44,-1,25,32,77,-1,25,83,32,-1,25,26,83,-1,25,27,26,-1,25,84,27,-1,25,44,84,-1,11,10,30,-1,11,30,13,-1,11,13,15,-1,11,15,14,-1,11,14,42,-1,11,42,12,-1,15,13,16,-1,15,18,14,-1,15,16,76,-1,15,76,18,-1,76,16,17,-1,76,17,82,-1,76,82,19,-1,76,19,18,-1,22,18,19,-1,22,87,18,-1,22,27,84,-1,22,84,87,-1,87,84,85,-1,85,84,44,-1,85,42,14,-1,87,14,18,-1,87,85,14,-1,20,83,26,-1,20,17,16,-1,20,16,88,-1,20,88,83,-1,88,16,13,-1,88,13,86,-1,88,86,83,-1,86,13,30,-1,86,32,83,-1,23,89,22,-1,89,27,22,-1,89,91,27,-1,91,26,27,-1,91,20,26,-1,21,20,91,-1,21,17,20,-1,21,92,17,-1,82,17,92,-1,82,90,19,-1,23,22,19,-1,23,19,90,-1,82,92,101,-1,82,101,99,-1,82,99,93,-1,82,93,95,-1,82,95,97,-1,82,97,90,-1,23,90,97,-1,23,97,94,-1,23,94,89,-1,89,94,96,-1,89,96,95,-1,89,95,93,-1,89,93,91,-1,91,93,99,-1,91,99,100,-1,91,100,98,-1,21,91,98,-1,21,98,101,-1,21,101,92,-1,85,105,42,-1,85,103,105,-1,85,44,103,-1,103,44,104,-1,80,42,105,-1,80,105,102,-1,80,102,104,-1,80,104,44,-1,105,109,102,-1,102,109,47,-1,47,104,102,-1,104,47,45,-1,104,45,103,-1,103,45,46,-1,103,46,109,-1,103,109,105,-1,109,112,110,-1,109,110,47,-1,47,110,111,-1,47,111,45,-1,45,111,113,-1,113,46,45,-1,46,113,112,-1,112,109,46,-1,112,118,110,-1,110,118,115,-1,110,115,111,-1,111,115,117,-1,111,117,113,-1,113,117,116,-1,113,116,112,-1,112,116,118,-1,115,118,119,-1,119,118,122,-1,118,116,122,-1,122,116,120,-1,116,117,120,-1,120,117,121,-1,117,115,121,-1,115,119,121,-1,119,127,123,-1,119,122,127,-1,122,126,127,-1,122,128,126,-1,122,120,128,-1,120,124,128,-1,120,121,124,-1,121,125,124,-1,121,119,125,-1,119,123,125,-1,127,129,123,-1,127,126,129,-1,129,126,141,-1,141,126,143,-1,126,142,143,-1,126,128,142,-1,128,124,130,-1,142,128,130,-1,124,132,130,-1,124,134,132,-1,125,134,124,-1,125,136,134,-1,125,137,136,-1,125,135,137,-1,125,133,135,-1,125,123,133,-1,123,131,133,-1,123,129,131,-1,131,129,138,-1,129,141,138,-1,138,141,144,-1,141,143,144,-1,143,146,144,-1,142,146,143,-1,142,145,146,-1,139,145,142,-1,130,139,142,-1,139,130,132,-1,139,132,154,-1,132,157,154,-1,132,159,157,-1,132,134,159,-1,134,136,159,-1,136,161,159,-1,136,137,161,-1,137,162,161,-1,160,162,137,-1,135,160,137,-1,133,160,135,-1,133,158,160,-1,131,158,133,-1,156,158,131,-1,153,156,131,-1,131,138,153,-1,138,155,153,-1,140,155,138,-1,138,144,140,-1,144,147,140,-1,140,147,145,-1,140,145,139,-1,139,155,140,-1,154,155,139,-1,146,149,144,-1,146,151,149,-1,145,151,146,-1,150,151,145,-1,145,152,150,-1,147,152,145,-1,147,149,152,-1,147,144,149,-1,148,149,151,-1,148,152,149,-1,148,150,152,-1,148,151,150,-1,160,207,162,-1,160,205,207,-1,165,208,205,-1,160,165,205,-1,158,165,160,-1,161,162,207,-1,161,207,206,-1,165,206,208,-1,206,165,161,-1,161,165,159,-1,207,209,211,-1,205,209,207,-1,205,212,209,-1,205,208,212,-1,206,212,208,-1,206,210,212,-1,206,207,210,-1,207,211,210,-1,209,212,213,-1,212,216,213,-1,212,214,216,-1,210,214,212,-1,210,215,214,-1,210,211,215,-1,209,215,211,-1,209,213,215,-1,217,213,216,-1,217,215,213,-1,217,214,215,-1,217,216,214,-1,158,194,165,-1,192,194,158,-1,164,195,192,-1,158,164,192,-1,156,164,158,-1,159,194,165,-1,159,194,193,-1,159,193,195,-1,159,195,164,-1,159,164,157,-1,157,164,180,-1,192,198,194,-1,192,196,198,-1,192,195,196,-1,195,199,196,-1,196,199,200,-1,199,203,200,-1,193,199,195,-1,193,197,199,-1,193,198,197,-1,193,194,198,-1,199,201,203,-1,197,201,199,-1,197,198,201,-1,198,202,201,-1,196,202,198,-1,200,202,196,-1,204,202,200,-1,204,201,202,-1,204,203,201,-1,204,200,203,-1,156,181,164,-1,156,179,181,-1,156,182,179,-1,156,163,182,-1,163,180,182,-1,157,180,163,-1,164,181,180,-1,179,182,183,-1,182,186,183,-1,182,184,186,-1,180,184,182,-1,180,181,184,-1,181,185,184,-1,179,185,181,-1,183,185,179,-1,183,186,187,-1,186,190,187,-1,184,190,186,-1,184,188,190,-1,184,185,188,-1,185,189,188,-1,185,183,189,-1,183,187,189,-1,191,189,187,-1,191,188,189,-1,191,190,188,-1,191,187,190,-1,153,163,156,-1,153,168,163,-1,153,166,168,-1,153,169,166,-1,155,169,153,-1,155,167,169,-1,154,167,155,-1,154,163,167,-1,154,157,163,-1,163,168,167,-1,166,169,170,-1,169,173,170,-1,169,171,173,-1,169,167,171,-1,167,168,171,-1,168,172,171,-1,168,170,172,-1,170,168,166,-1,170,173,174,-1,173,177,174,-1,173,175,177,-1,173,171,175,-1,171,172,175,-1,172,176,175,-1,172,174,176,-1,170,174,172,-1,178,176,174,-1,178,175,176,-1,178,177,175,-1,178,174,177,-1,86,30,221,-1,86,221,219,-1,86,219,32,-1,32,219,220,-1,78,32,220,-1,78,220,218,-1,78,218,221,-1,78,221,30,-1,221,225,219,-1,219,225,35,-1,35,33,219,-1,33,220,219,-1,33,34,220,-1,220,34,218,-1,221,218,34,-1,34,225,221,-1,225,226,228,-1,225,228,35,-1,35,228,229,-1,35,229,33,-1,33,229,227,-1,33,227,34,-1,34,227,226,-1,34,226,225,-1,226,234,228,-1,228,234,232,-1,232,229,228,-1,232,233,229,-1,229,233,227,-1,227,233,231,-1,227,231,226,-1,226,231,234,-1,231,235,234,-1,235,238,234,-1,234,238,232,-1,238,236,232,-1,232,236,233,-1,236,237,233,-1,233,237,231,-1,231,237,235,-1,235,239,243,-1,235,243,238,-1,238,243,242,-1,238,242,244,-1,238,244,236,-1,236,244,240,-1,236,240,237,-1,237,240,241,-1,237,241,235,-1,235,241,239,-1,243,239,245,-1,243,245,242,-1,245,257,242,-1,257,259,242,-1,242,259,258,-1,242,258,244,-1,244,246,240,-1,258,246,244,-1,240,246,248,-1,240,248,250,-1,241,240,250,-1,241,250,252,-1,241,252,253,-1,241,253,251,-1,241,251,249,-1,241,249,239,-1,239,249,247,-1,239,247,245,-1,247,254,245,-1,245,254,257,-1,254,260,257,-1,257,260,259,-1,259,260,262,-1,258,259,262,-1,258,262,261,-1,255,258,261,-1,246,258,255,-1,255,248,246,-1,255,270,248,-1,248,270,273,-1,248,273,275,-1,248,275,250,-1,250,275,252,-1,252,275,277,-1,252,277,253,-1,253,277,278,-1,276,253,278,-1,251,253,276,-1,249,251,276,-1,249,276,274,-1,247,249,274,-1,272,247,274,-1,269,247,272,-1,247,269,254,-1,254,269,271,-1,256,254,271,-1,254,256,260,-1,260,256,263,-1,256,261,263,-1,256,255,261,-1,255,256,271,-1,270,255,271,-1,262,260,265,-1,262,265,267,-1,261,262,267,-1,266,261,267,-1,261,266,268,-1,263,261,268,-1,263,268,265,-1,263,265,260,-1,264,267,265,-1,264,265,268,-1,264,268,266,-1,264,266,267,-1,276,278,323,-1,276,323,321,-1,281,321,324,-1,276,321,281,-1,274,276,281,-1,277,323,278,-1,277,322,323,-1,281,324,322,-1,322,277,281,-1,277,275,281,-1,323,327,325,-1,321,323,325,-1,321,325,328,-1,321,328,324,-1,322,324,328,-1,322,328,326,-1,322,326,323,-1,323,326,327,-1,325,329,328,-1,328,329,332,-1,328,332,330,-1,326,328,330,-1,326,330,331,-1,326,331,327,-1,325,327,331,-1,325,331,329,-1,333,332,329,-1,333,329,331,-1,333,331,330,-1,333,330,332,-1,274,281,310,-1,308,274,310,-1,280,308,311,-1,274,308,280,-1,272,274,280,-1,275,310,281,-1,275,309,310,-1,275,311,309,-1,275,280,311,-1,275,273,280,-1,273,296,280,-1,308,310,314,-1,308,314,312,-1,308,312,311,-1,311,312,315,-1,312,316,315,-1,315,316,319,-1,309,311,315,-1,309,315,313,-1,309,313,314,-1,309,314,310,-1,315,319,317,-1,313,315,317,-1,313,317,314,-1,314,317,318,-1,312,314,318,-1,316,312,318,-1,320,316,318,-1,320,318,317,-1,320,317,319,-1,320,319,316,-1,272,280,297,-1,272,297,295,-1,272,295,298,-1,272,298,279,-1,279,298,296,-1,273,279,296,-1,280,296,297,-1,295,299,298,-1,298,299,302,-1,298,302,300,-1,296,298,300,-1,296,300,297,-1,297,300,301,-1,295,297,301,-1,299,295,301,-1,299,303,302,-1,302,303,306,-1,300,302,306,-1,300,306,304,-1,300,304,301,-1,301,304,305,-1,301,305,299,-1,299,305,303,-1,307,303,305,-1,307,305,304,-1,307,304,306,-1,307,306,303,-1,269,272,279,-1,269,279,284,-1,269,284,282,-1,269,282,285,-1,271,269,285,-1,271,285,283,-1,270,271,283,-1,270,283,279,-1,270,279,273,-1,279,283,284,-1,282,286,285,-1,285,286,289,-1,285,289,287,-1,285,287,283,-1,283,287,284,-1,284,287,288,-1,284,288,286,-1,286,282,284,-1,286,290,289,-1,289,290,293,-1,289,293,291,-1,289,291,287,-1,287,291,288,-1,288,291,292,-1,288,292,290,-1,286,288,290,-1,294,290,292,-1,294,292,291,-1,294,291,293,-1,294,293,290,-1,97,334,336,-1,97,336,94,-1,94,336,96,-1,336,335,96,-1,96,335,95,-1,95,335,337,-1,95,337,334,-1,95,334,97,-1,334,341,336,-1,336,341,338,-1,336,338,335,-1,335,338,340,-1,335,340,337,-1,337,340,339,-1,337,339,334,-1,334,339,341,-1,341,345,342,-1,341,342,338,-1,338,342,340,-1,340,342,344,-1,340,344,339,-1,339,344,343,-1,339,343,345,-1,339,345,341,-1,345,349,342,-1,342,349,351,-1,342,351,346,-1,342,346,344,-1,71,346,348,-1,71,344,346,-1,71,348,347,-1,71,347,344,-1,344,347,343,-1,343,347,352,-1,343,352,349,-1,343,349,345,-1,349,352,356,-1,349,356,353,-1,349,353,355,-1,349,355,351,-1,354,356,352,-1,354,352,350,-1,354,350,351,-1,354,351,355,-1,353,356,357,-1,353,357,358,-1,353,358,359,-1,353,359,360,-1,353,360,361,-1,353,361,355,-1,354,357,356,-1,350,346,351,-1,348,346,347,-1,350,347,346,-1,350,352,347,-1,354,358,357,-1,354,359,358,-1,354,360,359,-1,354,361,360,-1,354,355,361,-1,101,362,365,-1,101,365,99,-1,99,365,100,-1,100,365,363,-1,100,363,98,-1,98,363,364,-1,98,364,101,-1,101,364,362,-1,362,369,367,-1,362,367,365,-1,365,367,363,-1,363,367,368,-1,363,367,368,-1,363,368,366,-1,363,366,364,-1,364,366,362,-1,362,366,369,-1,369,373,371,-1,369,371,367,-1,367,371,368,-1,368,371,372,-1,368,372,366,-1,366,372,370,-1,366,370,369,-1,369,370,373,-1,373,377,380,-1,373,380,375,-1,373,375,371,-1,371,375,372,-1,372,375,376,-1,372,376,374,-1,372,374,370,-1,370,374,379,-1,373,370,379,-1,373,379,377,-1,377,379,383,-1,377,383,381,-1,377,381,384,-1,377,384,380,-1,381,383,389,-1,381,389,388,-1,381,388,387,-1,381,387,386,-1,381,386,385,-1,381,385,384,-1,376,375,374,-1,378,379,374,-1,378,374,375,-1,378,375,380,-1,382,386,387,-1,382,387,388,-1,382,388,389,-1,382,389,383,-1,382,383,379,-1,382,379,378,-1,382,378,380,-1,382,380,384,-1,382,384,385,-1,382,385,386,-1]
TextureCoordinate136 = x3d.TextureCoordinate()
TextureCoordinate136.point = [(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5),(0.5, 0),(0, 0.5),(0, 0),(0.5, 0.5)]

IndexedFaceSet135.texCoord = TextureCoordinate136
Coordinate137 = x3d.Coordinate()
Coordinate137.USE = "Joe_SkinCoord"

IndexedFaceSet135.coord = Coordinate137

Shape130.geometry = IndexedFaceSet135

HAnimHumanoid25.skin.append(Shape130)

Group24.children.append(HAnimHumanoid25)

Scene13.children.append(Group24)
Group138 = x3d.Group()
TimeSensor139 = x3d.TimeSensor()
TimeSensor139.DEF = "KickTimer"
TimeSensor139.cycleInterval = 3.73
TimeSensor139.loop = True

Group138.children.append(TimeSensor139)
OrientationInterpolator140 = x3d.OrientationInterpolator()
OrientationInterpolator140.DEF = "HumanoidRootRotInterp"
OrientationInterpolator140.key = [0,0.1,0.4,0.6,1]
OrientationInterpolator140.keyValue = [(1, 0, 0, 0.5),(1, 0, 0, 0.5),(-1, 0, 0, 0.1),(-1, 0, 0, 0.5),(-1, 0, 0, 0.5)]

Group138.children.append(OrientationInterpolator140)
PositionInterpolator141 = x3d.PositionInterpolator()
PositionInterpolator141.DEF = "HumanoidRootTransInterp"
PositionInterpolator141.key = [0,0.2,0.6,1]
PositionInterpolator141.keyValue = [(1, 0.3, -1),(0.4, -0.04, -0.4),(-0.18, 0.1, 0),(-0.2, 0.15, 0.15)]

Group138.children.append(PositionInterpolator141)
OrientationInterpolator142 = x3d.OrientationInterpolator()
OrientationInterpolator142.DEF = "sacroiliacRotInterp"
OrientationInterpolator142.key = [0,0.5,1]
OrientationInterpolator142.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator142)
OrientationInterpolator143 = x3d.OrientationInterpolator()
OrientationInterpolator143.DEF = "l_hipRotInterp"
OrientationInterpolator143.key = [0,0.1,0.3,0.45,1]
OrientationInterpolator143.keyValue = [(-1, 0, 0, 1.5),(-1, 0, 0, 1),(0, 0, 1, 0),(1, 0, 0, 0.5),(1, 0, 0, 1)]

Group138.children.append(OrientationInterpolator143)
OrientationInterpolator144 = x3d.OrientationInterpolator()
OrientationInterpolator144.DEF = "l_kneeRotInterp"
OrientationInterpolator144.key = [0,0.2,0.35,0.5,1]
OrientationInterpolator144.keyValue = [(1, 0, 0, 1),(0, 0, 1, 0),(0, 0, 1, 0.2),(1, 0, 1, 0.5),(1, 0, 0, 1.4)]

Group138.children.append(OrientationInterpolator144)
OrientationInterpolator145 = x3d.OrientationInterpolator()
OrientationInterpolator145.DEF = "l_ankleRotInterp"
OrientationInterpolator145.key = [0,0.25,1]
OrientationInterpolator145.keyValue = [(-1, 0, 0, 1),(0, 0, 1, 0),(1, 0, 0, 1)]

Group138.children.append(OrientationInterpolator145)
OrientationInterpolator146 = x3d.OrientationInterpolator()
OrientationInterpolator146.DEF = "l_subtalarRotInterp"
OrientationInterpolator146.key = [0,0.5,1]
OrientationInterpolator146.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator146)
OrientationInterpolator147 = x3d.OrientationInterpolator()
OrientationInterpolator147.DEF = "l_midtarsalRotInterp"
OrientationInterpolator147.key = [0,0.5,1]
OrientationInterpolator147.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator147)
OrientationInterpolator148 = x3d.OrientationInterpolator()
OrientationInterpolator148.DEF = "l_metatarsalRotInterp"
OrientationInterpolator148.key = [0,0.5,1]
OrientationInterpolator148.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator148)
OrientationInterpolator149 = x3d.OrientationInterpolator()
OrientationInterpolator149.DEF = "r_hipRotInterp"
OrientationInterpolator149.key = [0,0.25,0.5,0.75,1]
OrientationInterpolator149.keyValue = [(1, 0, 0, 1),(1, 0, 0, 1),(-1, 0, 0, 1),(-1, 0, 0, 1),(-1, 0, 0, 1)]

Group138.children.append(OrientationInterpolator149)
OrientationInterpolator150 = x3d.OrientationInterpolator()
OrientationInterpolator150.DEF = "r_kneeRotInterp"
OrientationInterpolator150.key = [0,0.25,0.5,0.75,1]
OrientationInterpolator150.keyValue = [(1, 0, 0, 0.1),(0, 0, 1, 0),(1, 0, 0, 1),(1, 0, 0, 1),(1, 0, 0, 1.5)]

Group138.children.append(OrientationInterpolator150)
OrientationInterpolator151 = x3d.OrientationInterpolator()
OrientationInterpolator151.DEF = "r_ankleRotInterp"
OrientationInterpolator151.key = [0,0.25,0.5,0.75,1]
OrientationInterpolator151.keyValue = [(-1, 0, 0, 1),(0, 0, 1, 0),(1, 0, 0, 1),(1, 0, 0, 1),(1, 0, 0, 0.5)]

Group138.children.append(OrientationInterpolator151)
OrientationInterpolator152 = x3d.OrientationInterpolator()
OrientationInterpolator152.DEF = "r_subtalarRotInterp"
OrientationInterpolator152.key = [0,0.5,1]
OrientationInterpolator152.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator152)
OrientationInterpolator153 = x3d.OrientationInterpolator()
OrientationInterpolator153.DEF = "r_midtarsalRotInterp"
OrientationInterpolator153.key = [0,0.5,1]
OrientationInterpolator153.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator153)
OrientationInterpolator154 = x3d.OrientationInterpolator()
OrientationInterpolator154.DEF = "r_metatarsalRotInterp"
OrientationInterpolator154.key = [0,0.5,1]
OrientationInterpolator154.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator154)
OrientationInterpolator155 = x3d.OrientationInterpolator()
OrientationInterpolator155.DEF = "vl5RotInterp"
OrientationInterpolator155.key = [0,0.5,1]
OrientationInterpolator155.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator155)
OrientationInterpolator156 = x3d.OrientationInterpolator()
OrientationInterpolator156.DEF = "vl4RotInterp"
OrientationInterpolator156.key = [0,0.5,1]
OrientationInterpolator156.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator156)
OrientationInterpolator157 = x3d.OrientationInterpolator()
OrientationInterpolator157.DEF = "vl3RotInterp"
OrientationInterpolator157.key = [0,0.5,1]
OrientationInterpolator157.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator157)
OrientationInterpolator158 = x3d.OrientationInterpolator()
OrientationInterpolator158.DEF = "vl2RotInterp"
OrientationInterpolator158.key = [0,0.5,1]
OrientationInterpolator158.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator158)
OrientationInterpolator159 = x3d.OrientationInterpolator()
OrientationInterpolator159.DEF = "vl1RotInterp"
OrientationInterpolator159.key = [0,0.5,1]
OrientationInterpolator159.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator159)
OrientationInterpolator160 = x3d.OrientationInterpolator()
OrientationInterpolator160.DEF = "vt12RotInterp"
OrientationInterpolator160.key = [0,0.5,1]
OrientationInterpolator160.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator160)
OrientationInterpolator161 = x3d.OrientationInterpolator()
OrientationInterpolator161.DEF = "vt11RotInterp"
OrientationInterpolator161.key = [0,0.5,1]
OrientationInterpolator161.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator161)
OrientationInterpolator162 = x3d.OrientationInterpolator()
OrientationInterpolator162.DEF = "vt10RotInterp"
OrientationInterpolator162.key = [0,0.5,1]
OrientationInterpolator162.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator162)
OrientationInterpolator163 = x3d.OrientationInterpolator()
OrientationInterpolator163.DEF = "vt9RotInterp"
OrientationInterpolator163.key = [0,0.5,1]
OrientationInterpolator163.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator163)
OrientationInterpolator164 = x3d.OrientationInterpolator()
OrientationInterpolator164.DEF = "vt8RotInterp"
OrientationInterpolator164.key = [0,0.5,1]
OrientationInterpolator164.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator164)
OrientationInterpolator165 = x3d.OrientationInterpolator()
OrientationInterpolator165.DEF = "vt7RotInterp"
OrientationInterpolator165.key = [0,0.5,1]
OrientationInterpolator165.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator165)
OrientationInterpolator166 = x3d.OrientationInterpolator()
OrientationInterpolator166.DEF = "vt6RotInterp"
OrientationInterpolator166.key = [0,0.5,1]
OrientationInterpolator166.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator166)
OrientationInterpolator167 = x3d.OrientationInterpolator()
OrientationInterpolator167.DEF = "vt5RotInterp"
OrientationInterpolator167.key = [0,0.5,1]
OrientationInterpolator167.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator167)
OrientationInterpolator168 = x3d.OrientationInterpolator()
OrientationInterpolator168.DEF = "vt4RotInterp"
OrientationInterpolator168.key = [0,0.5,1]
OrientationInterpolator168.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator168)
OrientationInterpolator169 = x3d.OrientationInterpolator()
OrientationInterpolator169.DEF = "vt3RotInterp"
OrientationInterpolator169.key = [0,0.5,1]
OrientationInterpolator169.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator169)
OrientationInterpolator170 = x3d.OrientationInterpolator()
OrientationInterpolator170.DEF = "vt2RotInterp"
OrientationInterpolator170.key = [0,0.5,1]
OrientationInterpolator170.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator170)
OrientationInterpolator171 = x3d.OrientationInterpolator()
OrientationInterpolator171.DEF = "vt1RotInterp"
OrientationInterpolator171.key = [0,0.5,1]
OrientationInterpolator171.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator171)
OrientationInterpolator172 = x3d.OrientationInterpolator()
OrientationInterpolator172.DEF = "vc7RotInterp"
OrientationInterpolator172.key = [0,0.5,1]
OrientationInterpolator172.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator172)
OrientationInterpolator173 = x3d.OrientationInterpolator()
OrientationInterpolator173.DEF = "vc6RotInterp"
OrientationInterpolator173.key = [0,0.5,1]
OrientationInterpolator173.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator173)
OrientationInterpolator174 = x3d.OrientationInterpolator()
OrientationInterpolator174.DEF = "vc5RotInterp"
OrientationInterpolator174.key = [0,0.5,1]
OrientationInterpolator174.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator174)
OrientationInterpolator175 = x3d.OrientationInterpolator()
OrientationInterpolator175.DEF = "vc4RotInterp"
OrientationInterpolator175.key = [0,0.3,0.4,1]
OrientationInterpolator175.keyValue = [(1, 0, 1, 0.25),(-1, 0, -1, 0.35),(1, 0, 0, 0.75),(1, 0, 1, 0.5)]

Group138.children.append(OrientationInterpolator175)
OrientationInterpolator176 = x3d.OrientationInterpolator()
OrientationInterpolator176.DEF = "vc3RotInterp"
OrientationInterpolator176.key = [0,0.5,1]
OrientationInterpolator176.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator176)
OrientationInterpolator177 = x3d.OrientationInterpolator()
OrientationInterpolator177.DEF = "vc2RotInterp"
OrientationInterpolator177.key = [0,0.5,1]
OrientationInterpolator177.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator177)
OrientationInterpolator178 = x3d.OrientationInterpolator()
OrientationInterpolator178.DEF = "vc1RotInterp"
OrientationInterpolator178.key = [0,0.5,1]
OrientationInterpolator178.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator178)
OrientationInterpolator179 = x3d.OrientationInterpolator()
OrientationInterpolator179.DEF = "skullbaseRotInterp"
OrientationInterpolator179.key = [0,0.2,0.75,1]
OrientationInterpolator179.keyValue = [(0, -1, 0, 0.5),(0, 0, 1, 0),(0, 0, 1, 0),(0, 1, 0, 0.35)]

Group138.children.append(OrientationInterpolator179)
OrientationInterpolator180 = x3d.OrientationInterpolator()
OrientationInterpolator180.DEF = "l_eyelid_jointRotInterp"
OrientationInterpolator180.key = [0,0.5,1]
OrientationInterpolator180.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator180)
OrientationInterpolator181 = x3d.OrientationInterpolator()
OrientationInterpolator181.DEF = "l_eyeball_jointRotInterp"
OrientationInterpolator181.key = [0,0.5,1]
OrientationInterpolator181.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator181)
OrientationInterpolator182 = x3d.OrientationInterpolator()
OrientationInterpolator182.DEF = "l_eyebrow_jointRotInterp"
OrientationInterpolator182.key = [0,0.5,1]
OrientationInterpolator182.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator182)
OrientationInterpolator183 = x3d.OrientationInterpolator()
OrientationInterpolator183.DEF = "r_eyelid_jointRotInterp"
OrientationInterpolator183.key = [0,0.5,1]
OrientationInterpolator183.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator183)
OrientationInterpolator184 = x3d.OrientationInterpolator()
OrientationInterpolator184.DEF = "r_eyeball_jointRotInterp"
OrientationInterpolator184.key = [0,0.5,1]
OrientationInterpolator184.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator184)
OrientationInterpolator185 = x3d.OrientationInterpolator()
OrientationInterpolator185.DEF = "r_eyebrow_jointRotInterp"
OrientationInterpolator185.key = [0,0.5,1]
OrientationInterpolator185.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator185)
OrientationInterpolator186 = x3d.OrientationInterpolator()
OrientationInterpolator186.DEF = "temporomandibularRotInterp"
OrientationInterpolator186.key = [0,0.5,1]
OrientationInterpolator186.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator186)
OrientationInterpolator187 = x3d.OrientationInterpolator()
OrientationInterpolator187.DEF = "l_sternoclavicularRotInterp"
OrientationInterpolator187.key = [0,0.5,1]
OrientationInterpolator187.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator187)
OrientationInterpolator188 = x3d.OrientationInterpolator()
OrientationInterpolator188.DEF = "l_acromioclavicularRotInterp"
OrientationInterpolator188.key = [0,0.5,1]
OrientationInterpolator188.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator188)
OrientationInterpolator189 = x3d.OrientationInterpolator()
OrientationInterpolator189.DEF = "l_shoulderRotInterp"
OrientationInterpolator189.key = [0,0.4,1]
OrientationInterpolator189.keyValue = [(0, 0, 1, 0),(0, 0, 1, 1.5),(-1, 0, 1, 1.75)]

Group138.children.append(OrientationInterpolator189)
OrientationInterpolator190 = x3d.OrientationInterpolator()
OrientationInterpolator190.DEF = "l_elbowRotInterp"
OrientationInterpolator190.key = [0,0.5,1]
OrientationInterpolator190.keyValue = [(-1, 0, 0, 3),(-1, 0, 0, 0.75),(-1, -1, 0, 0.5)]

Group138.children.append(OrientationInterpolator190)
OrientationInterpolator191 = x3d.OrientationInterpolator()
OrientationInterpolator191.DEF = "l_wristRotInterp"
OrientationInterpolator191.key = [0,0.4,0.8,1]
OrientationInterpolator191.keyValue = [(0, 0, 1, 0),(0, 1, 0, 1.3),(0, -0.5, 1, 1.3),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator191)
OrientationInterpolator192 = x3d.OrientationInterpolator()
OrientationInterpolator192.DEF = "l_thumb1RotInterp"
OrientationInterpolator192.key = [0,0.5,1]
OrientationInterpolator192.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator192)
OrientationInterpolator193 = x3d.OrientationInterpolator()
OrientationInterpolator193.DEF = "l_thumb2RotInterp"
OrientationInterpolator193.key = [0,0.5,1]
OrientationInterpolator193.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator193)
OrientationInterpolator194 = x3d.OrientationInterpolator()
OrientationInterpolator194.DEF = "l_thumb3RotInterp"
OrientationInterpolator194.key = [0,0.5,1]
OrientationInterpolator194.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator194)
OrientationInterpolator195 = x3d.OrientationInterpolator()
OrientationInterpolator195.DEF = "l_index0RotInterp"
OrientationInterpolator195.key = [0,0.5,1]
OrientationInterpolator195.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator195)
OrientationInterpolator196 = x3d.OrientationInterpolator()
OrientationInterpolator196.DEF = "l_index1RotInterp"
OrientationInterpolator196.key = [0,0.5,1]
OrientationInterpolator196.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator196)
OrientationInterpolator197 = x3d.OrientationInterpolator()
OrientationInterpolator197.DEF = "l_index2RotInterp"
OrientationInterpolator197.key = [0,0.5,1]
OrientationInterpolator197.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator197)
OrientationInterpolator198 = x3d.OrientationInterpolator()
OrientationInterpolator198.DEF = "l_index3RotInterp"
OrientationInterpolator198.key = [0,0.5,1]
OrientationInterpolator198.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator198)
OrientationInterpolator199 = x3d.OrientationInterpolator()
OrientationInterpolator199.DEF = "l_middle0RotInterp"
OrientationInterpolator199.key = [0,0.5,1]
OrientationInterpolator199.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator199)
OrientationInterpolator200 = x3d.OrientationInterpolator()
OrientationInterpolator200.DEF = "l_middle1RotInterp"
OrientationInterpolator200.key = [0,0.5,1]
OrientationInterpolator200.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator200)
OrientationInterpolator201 = x3d.OrientationInterpolator()
OrientationInterpolator201.DEF = "l_middle2RotInterp"
OrientationInterpolator201.key = [0,0.5,1]
OrientationInterpolator201.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator201)
OrientationInterpolator202 = x3d.OrientationInterpolator()
OrientationInterpolator202.DEF = "l_middle3RotInterp"
OrientationInterpolator202.key = [0,0.5,1]
OrientationInterpolator202.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator202)
OrientationInterpolator203 = x3d.OrientationInterpolator()
OrientationInterpolator203.DEF = "l_ring0RotInterp"
OrientationInterpolator203.key = [0,0.5,1]
OrientationInterpolator203.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator203)
OrientationInterpolator204 = x3d.OrientationInterpolator()
OrientationInterpolator204.DEF = "l_ring1RotInterp"
OrientationInterpolator204.key = [0,0.5,1]
OrientationInterpolator204.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator204)
OrientationInterpolator205 = x3d.OrientationInterpolator()
OrientationInterpolator205.DEF = "l_ring2RotInterp"
OrientationInterpolator205.key = [0,0.5,1]
OrientationInterpolator205.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator205)
OrientationInterpolator206 = x3d.OrientationInterpolator()
OrientationInterpolator206.DEF = "l_ring3RotInterp"
OrientationInterpolator206.key = [0,0.5,1]
OrientationInterpolator206.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator206)
OrientationInterpolator207 = x3d.OrientationInterpolator()
OrientationInterpolator207.DEF = "l_pinky0RotInterp"
OrientationInterpolator207.key = [0,0.5,1]
OrientationInterpolator207.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator207)
OrientationInterpolator208 = x3d.OrientationInterpolator()
OrientationInterpolator208.DEF = "l_pinky1RotInterp"
OrientationInterpolator208.key = [0,0.5,1]
OrientationInterpolator208.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator208)
OrientationInterpolator209 = x3d.OrientationInterpolator()
OrientationInterpolator209.DEF = "l_pinky2RotInterp"
OrientationInterpolator209.key = [0,0.5,1]
OrientationInterpolator209.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator209)
OrientationInterpolator210 = x3d.OrientationInterpolator()
OrientationInterpolator210.DEF = "l_pinky3RotInterp"
OrientationInterpolator210.key = [0,0.5,1]
OrientationInterpolator210.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator210)
OrientationInterpolator211 = x3d.OrientationInterpolator()
OrientationInterpolator211.DEF = "r_sternoclavicularRotInterp"
OrientationInterpolator211.key = [0,0.5,1]
OrientationInterpolator211.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator211)
OrientationInterpolator212 = x3d.OrientationInterpolator()
OrientationInterpolator212.DEF = "r_acromioclavicularRotInterp"
OrientationInterpolator212.key = [0,0.5,1]
OrientationInterpolator212.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator212)
OrientationInterpolator213 = x3d.OrientationInterpolator()
OrientationInterpolator213.DEF = "r_shoulderRotInterp"
OrientationInterpolator213.key = [0,0.5,1]
OrientationInterpolator213.keyValue = [(0, 0, -1, 2.5),(0, 0, -1, 1.5),(0, 0, -1, 1.75)]

Group138.children.append(OrientationInterpolator213)
OrientationInterpolator214 = x3d.OrientationInterpolator()
OrientationInterpolator214.DEF = "r_elbowRotInterp"
OrientationInterpolator214.key = [0,0.5,1]
OrientationInterpolator214.keyValue = [(-1, 0, 0, 3),(-1, 0, 0, 0.75),(-1, -1, 0, 0.5)]

Group138.children.append(OrientationInterpolator214)
OrientationInterpolator215 = x3d.OrientationInterpolator()
OrientationInterpolator215.DEF = "r_wristRotInterp"
OrientationInterpolator215.key = [0,0.5,0.7,1]
OrientationInterpolator215.keyValue = [(0, 1, 0, 0.3),(0, 0, 1, 0),(0, 0, -1, 1),(0, -1, 0, 0.3)]

Group138.children.append(OrientationInterpolator215)
OrientationInterpolator216 = x3d.OrientationInterpolator()
OrientationInterpolator216.DEF = "r_thumb1RotInterp"
OrientationInterpolator216.key = [0,0.5,1]
OrientationInterpolator216.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator216)
OrientationInterpolator217 = x3d.OrientationInterpolator()
OrientationInterpolator217.DEF = "r_thumb2RotInterp"
OrientationInterpolator217.key = [0,0.5,1]
OrientationInterpolator217.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator217)
OrientationInterpolator218 = x3d.OrientationInterpolator()
OrientationInterpolator218.DEF = "r_thumb3RotInterp"
OrientationInterpolator218.key = [0,0.5,1]
OrientationInterpolator218.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator218)
OrientationInterpolator219 = x3d.OrientationInterpolator()
OrientationInterpolator219.DEF = "r_index0RotInterp"
OrientationInterpolator219.key = [0,0.5,0.75,1]
OrientationInterpolator219.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 1),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator219)
OrientationInterpolator220 = x3d.OrientationInterpolator()
OrientationInterpolator220.DEF = "r_index1RotInterp"
OrientationInterpolator220.key = [0,0.5,0.75,1]
OrientationInterpolator220.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 1.5),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator220)
OrientationInterpolator221 = x3d.OrientationInterpolator()
OrientationInterpolator221.DEF = "r_index2RotInterp"
OrientationInterpolator221.key = [0,0.5,0.75,1]
OrientationInterpolator221.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 1.5),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator221)
OrientationInterpolator222 = x3d.OrientationInterpolator()
OrientationInterpolator222.DEF = "r_index3RotInterp"
OrientationInterpolator222.key = [0,0.5,0.75,1]
OrientationInterpolator222.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 1.5),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator222)
OrientationInterpolator223 = x3d.OrientationInterpolator()
OrientationInterpolator223.DEF = "r_middle0RotInterp"
OrientationInterpolator223.key = [0,0.5,0.75,1]
OrientationInterpolator223.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 1),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator223)
OrientationInterpolator224 = x3d.OrientationInterpolator()
OrientationInterpolator224.DEF = "r_middle1RotInterp"
OrientationInterpolator224.key = [0,0.5,0.75,1]
OrientationInterpolator224.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 1.5),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator224)
OrientationInterpolator225 = x3d.OrientationInterpolator()
OrientationInterpolator225.DEF = "r_middle2RotInterp"
OrientationInterpolator225.key = [0,0.5,0.75,1]
OrientationInterpolator225.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 1.5),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator225)
OrientationInterpolator226 = x3d.OrientationInterpolator()
OrientationInterpolator226.DEF = "r_middle3RotInterp"
OrientationInterpolator226.key = [0,0.5,0.75,1]
OrientationInterpolator226.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 1.5),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator226)
OrientationInterpolator227 = x3d.OrientationInterpolator()
OrientationInterpolator227.DEF = "r_ring0RotInterp"
OrientationInterpolator227.key = [0,0.5,0.75,1]
OrientationInterpolator227.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 1),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator227)
OrientationInterpolator228 = x3d.OrientationInterpolator()
OrientationInterpolator228.DEF = "r_ring1RotInterp"
OrientationInterpolator228.key = [0,0.5,0.75,1]
OrientationInterpolator228.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 1.5),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator228)
OrientationInterpolator229 = x3d.OrientationInterpolator()
OrientationInterpolator229.DEF = "r_ring2RotInterp"
OrientationInterpolator229.key = [0,0.5,0.75,1]
OrientationInterpolator229.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 1.5),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator229)
OrientationInterpolator230 = x3d.OrientationInterpolator()
OrientationInterpolator230.DEF = "r_ring3RotInterp"
OrientationInterpolator230.key = [0,0.5,0.75,1]
OrientationInterpolator230.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 1.5),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator230)
OrientationInterpolator231 = x3d.OrientationInterpolator()
OrientationInterpolator231.DEF = "r_pinky0RotInterp"
OrientationInterpolator231.key = [0,0.5,0.75,1]
OrientationInterpolator231.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 1),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator231)
OrientationInterpolator232 = x3d.OrientationInterpolator()
OrientationInterpolator232.DEF = "r_pinky1RotInterp"
OrientationInterpolator232.key = [0,0.5,0.75,1]
OrientationInterpolator232.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 1.5),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator232)
OrientationInterpolator233 = x3d.OrientationInterpolator()
OrientationInterpolator233.DEF = "r_pinky2RotInterp"
OrientationInterpolator233.key = [0,0.5,0.75,1]
OrientationInterpolator233.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 1.5),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator233)
OrientationInterpolator234 = x3d.OrientationInterpolator()
OrientationInterpolator234.DEF = "r_pinky3RotInterp"
OrientationInterpolator234.key = [0,0.5,0.75,1]
OrientationInterpolator234.keyValue = [(0, 0, 1, 0),(0, 0, 1, 0),(0, 0, 1, 1.5),(0, 0, 1, 0)]

Group138.children.append(OrientationInterpolator234)

Scene13.children.append(Group138)
Group235 = x3d.Group()
Group235.DEF = "DisplacersAnimationGroup"
ScalarInterpolator236 = x3d.ScalarInterpolator()
ScalarInterpolator236.DEF = "skull_tipTest"
ScalarInterpolator236.key = [0,0.1,0.2,0.35,0.6,0.7,0.85,0.88,0.94,0.97,1]
ScalarInterpolator236.keyValue = [0,0,0,0,0.2,0.4,1,0,1,0.4,0]

Group235.children.append(ScalarInterpolator236)

Scene13.children.append(Group235)
Group237 = x3d.Group()
Group237.DEF = "skintexturetransform_animation"
ScalarInterpolator238 = x3d.ScalarInterpolator()
ScalarInterpolator238.DEF = "skinTexTransTest"
ScalarInterpolator238.key = [0,0.2,0.4,0.5,0.6,0.7,0.8,1]
ScalarInterpolator238.keyValue = [0,0,0,0,0,1,2,0]

Group237.children.append(ScalarInterpolator238)

Scene13.children.append(Group237)
Group239 = x3d.Group()
Transform240 = x3d.Transform()
Transform240.DEF = "SBall"
Transform240.translation = [-0.4564343,0.2206233,-0.4564343]
Transform240.rotation = [-0.707106781186546,0,-0.707106781186546,0.619705187235697]
Transform240.scale = [0.23,0.23,0.23]
Shape241 = x3d.Shape()
Shape241.DEF = "ball_Shape"
Appearance242 = x3d.Appearance()
Appearance242.DEF = "ball_Appearance"
Material243 = x3d.Material()
Material243.DEF = "ball_Material"
Material243.diffuseColor = [0.3,0.3,1]
Material243.emissiveColor = [0.3,0.3,0.33]

Appearance242.material = Material243
ImageTexture244 = x3d.ImageTexture()
ImageTexture244.USE = "JoeSkinImageTexture"

Appearance242.texture = ImageTexture244

Shape241.appearance = Appearance242
IndexedFaceSet245 = x3d.IndexedFaceSet()
IndexedFaceSet245.DEF = "ball_IndexedFaceSet"
IndexedFaceSet245.coordIndex = [0,1,2,-1,0,2,3,-1,0,3,4,-1,0,4,5,-1,0,5,6,-1,0,6,7,-1,0,7,8,-1,0,8,9,-1,0,9,10,-1,0,10,11,-1,0,11,12,-1,0,12,1,-1,1,13,14,-1,1,14,2,-1,2,14,15,-1,2,15,3,-1,3,15,16,-1,3,16,4,-1,4,16,17,-1,4,17,5,-1,5,17,18,-1,5,18,6,-1,6,18,19,-1,6,19,7,-1,7,19,20,-1,7,20,8,-1,8,20,21,-1,8,21,9,-1,9,21,22,-1,9,22,10,-1,10,22,23,-1,10,23,11,-1,11,23,24,-1,11,24,12,-1,12,24,13,-1,12,13,1,-1,13,25,26,-1,13,26,14,-1,14,26,27,-1,14,27,15,-1,15,27,28,-1,15,28,16,-1,16,28,29,-1,16,29,17,-1,17,29,30,-1,17,30,18,-1,18,30,31,-1,18,31,19,-1,19,31,32,-1,19,32,20,-1,20,32,33,-1,20,33,21,-1,21,33,34,-1,21,34,22,-1,22,34,35,-1,22,35,23,-1,23,35,36,-1,23,36,24,-1,24,36,25,-1,24,25,13,-1,25,37,38,-1,25,38,26,-1,26,38,39,-1,26,39,27,-1,27,39,40,-1,27,40,28,-1,28,40,41,-1,28,41,29,-1,29,41,42,-1,29,42,30,-1,30,42,43,-1,30,43,31,-1,31,43,44,-1,31,44,32,-1,32,44,45,-1,32,45,33,-1,33,45,46,-1,33,46,34,-1,34,46,47,-1,34,47,35,-1,35,47,48,-1,35,48,36,-1,36,48,37,-1,36,37,25,-1,37,49,50,-1,37,50,38,-1,38,50,51,-1,38,51,39,-1,39,51,52,-1,39,52,40,-1,40,52,53,-1,40,53,41,-1,41,53,54,-1,41,54,42,-1,42,54,55,-1,42,55,43,-1,43,55,56,-1,43,56,44,-1,44,56,57,-1,44,57,45,-1,45,57,58,-1,45,58,46,-1,46,58,59,-1,46,59,47,-1,47,59,60,-1,47,60,48,-1,48,60,49,-1,48,49,37,-1,61,50,49,-1,61,51,50,-1,61,52,51,-1,61,53,52,-1,61,54,53,-1,61,55,54,-1,61,56,55,-1,61,57,56,-1,61,58,57,-1,61,59,58,-1,61,60,59,-1,61,49,60,-1]
Coordinate246 = x3d.Coordinate()
Coordinate246.DEF = "Ball_Coordinates"
Coordinate246.point = [(0, 0.4675, 0),(0, 0.4049, -0.2338),(-0.1169, 0.4049, -0.2024),(-0.2024, 0.4049, -0.1169),(-0.2338, 0.4049, 0),(-0.2024, 0.4049, 0.1169),(-0.1169, 0.4049, 0.2024),(0, 0.4049, 0.2338),(0.1169, 0.4049, 0.2024),(0.2024, 0.4049, 0.1169),(0.2338, 0.4049, 0),(0.2024, 0.4049, -0.1169),(0.1169, 0.4049, -0.2024),(0, 0.2338, -0.4049),(-0.2024, 0.2338, -0.3506),(-0.3506, 0.2338, -0.2024),(-0.4049, 0.2338, 0),(-0.3506, 0.2338, 0.2024),(-0.2024, 0.2338, 0.3506),(0, 0.2338, 0.4049),(0.2024, 0.2338, 0.3506),(0.3506, 0.2338, 0.2024),(0.4049, 0.2338, 0),(0.3506, 0.2338, -0.2024),(0.2024, 0.2338, -0.3506),(0, 0, -0.4675),(-0.2338, 0, -0.4049),(-0.4049, 0, -0.2338),(-0.4675, 0, 0),(-0.4049, 0, 0.2338),(-0.2338, 0, 0.4049),(0, 0, 0.4675),(0.2338, 0, 0.4049),(0.4049, 0, 0.2338),(0.4675, 0, 0),(0.4049, 0, -0.2338),(0.2338, 0, -0.4049),(0, -0.2338, -0.4049),(-0.2024, -0.2338, -0.3506),(-0.3506, -0.2338, -0.2024),(-0.4049, -0.2338, 0),(-0.3506, -0.2338, 0.2024),(-0.2024, -0.2338, 0.3506),(0, -0.2338, 0.4049),(0.2024, -0.2338, 0.3506),(0.3506, -0.2338, 0.2024),(0.4049, -0.2338, 0),(0.3506, -0.2338, -0.2024),(0.2024, -0.2338, -0.3506),(0, -0.4049, -0.2338),(-0.1169, -0.4049, -0.2024),(-0.2024, -0.4049, -0.1169),(-0.2338, -0.4049, 0),(-0.2024, -0.4049, 0.1169),(-0.1169, -0.4049, 0.2024),(0, -0.4049, 0.2338),(0.1169, -0.4049, 0.2024),(0.2024, -0.4049, 0.1169),(0.2338, -0.4049, 0),(0.2024, -0.4049, -0.1169),(0.1169, -0.4049, -0.2024),(0, -0.4675, 0)]

IndexedFaceSet245.coord = Coordinate246

Shape241.geometry = IndexedFaceSet245

Transform240.children.append(Shape241)
Viewpoint247 = x3d.Viewpoint()
Viewpoint247.DEF = "ballView_1"
Viewpoint247.description = "Ball View"

Transform240.children.append(Viewpoint247)

Group239.children.append(Transform240)
PositionInterpolator248 = x3d.PositionInterpolator()
PositionInterpolator248.DEF = "ballTransInterp"
PositionInterpolator248.key = [0,0.4,0.409,1]
PositionInterpolator248.keyValue = [(-1, 0.4, -1),(0, 0.07, 0),(0.05, 0.06, 0.05),(2, 4, 10)]

Group239.children.append(PositionInterpolator248)
OrientationInterpolator249 = x3d.OrientationInterpolator()
OrientationInterpolator249.DEF = "ballRotInterp"
OrientationInterpolator249.key = [0,0.4,0.41,0.71,1]
OrientationInterpolator249.keyValue = [(1, 0, 1, 0.25),(-1, 0, -1, 1.35),(-1, 1, -1, 3.35),(-1, 0.2, -1, 3),(-1, 0.2, -1, 3)]

Group239.children.append(OrientationInterpolator249)

Scene13.children.append(Group239)
Group250 = x3d.Group()
Transform251 = x3d.Transform()
Transform251.scale = [0.2,0.2,0.2]
Shape252 = x3d.Shape()
Shape252.USE = "AxisLinesShape"

Transform251.children.append(Shape252)

Group250.children.append(Transform251)
Transform253 = x3d.Transform()
Transform253.DEF = "Circle0"
Transform253.scale = [1.175,1,1.175]
Shape254 = x3d.Shape()
Shape254.DEF = "circle_Shape"
Appearance255 = x3d.Appearance()
Appearance255.DEF = "circle0_Appearance"
Material256 = x3d.Material()
Material256.DEF = "circle0_Material"
Material256.ambientIntensity = 0.9
Material256.diffuseColor = [0.9,0,0.7]
Material256.emissiveColor = [0.425,0.486,1]

Appearance255.material = Material256

Shape254.appearance = Appearance255
IndexedLineSet257 = x3d.IndexedLineSet()
IndexedLineSet257.DEF = "Orbit1"
IndexedLineSet257.coordIndex = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,-1]
Coordinate258 = x3d.Coordinate()
Coordinate258.DEF = "circle_Coordinates"
Coordinate258.point = [(1, 0, 0),(0.995, 0, -0.105),(0.979, 0, -0.208),(0.951, 0, -0.309),(0.914, 0, -0.407),(0.866, 0, -0.5),(0.809, 0, -0.588),(0.743, 0, -0.669),(0.669, 0, -0.743),(0.588, 0, -0.809),(0.5, 0, -0.866),(0.407, 0, -0.914),(0.309, 0, -0.951),(0.208, 0, -0.978),(0.105, 0, -0.995),(0, 0, -1),(-0.105, 0, -0.994522),(-0.208, 0, -0.978),(-0.309, 0, -0.951),(-0.407, 0, -0.914),(-0.5, 0, -0.866),(-0.588, 0, -0.809),(-0.669, 0, -0.743),(-0.743, 0, -0.669),(-0.809, 0, -0.588),(-0.866, 0, -0.5),(-0.914, 0, -0.407),(-0.951, 0, -0.309),(-0.978, 0, -0.208),(-0.995, 0, -0.105),(-1, 0, 0),(-0.995, 0, 0.105),(-0.978, 0, 0.208),(-0.951, 0, 0.309),(-0.914, 0, 0.407),(-0.866, 0, 0.5),(-0.809, 0, 0.588),(-0.743, 0, 0.669),(-0.669, 0, 0.743),(-0.588, 0, 0.809),(-0.5, 0, 0.866),(-0.407, 0, 0.914),(-0.309, 0, 0.951),(-0.208, 0, 0.978),(-0.105, 0, 0.995),(0, 0, 1),(0.105, 0, 0.995),(0.208, 0, 0.978),(0.309, 0, 0.951),(0.407, 0, 0.914),(0.5, 0, 0.866),(0.588, 0, 0.809),(0.669, 0, 0.743),(0.743, 0, 0.669),(0.809, 0, 0.588),(0.866, 0, 0.5),(0.914, 0, 0.407),(0.951, 0, 0.309),(0.978, 0, 0.208),(0.995, 0, 0.104),(1, 0, 0)]

IndexedLineSet257.coord = Coordinate258

Shape254.geometry = IndexedLineSet257

Transform253.children.append(Shape254)

Group250.children.append(Transform253)
Transform259 = x3d.Transform()
Transform259.DEF = "Circle1"
Transform259.scale = [0.5,1,0.5]
Shape260 = x3d.Shape()
Shape260.DEF = "circle1_Shape"
Appearance261 = x3d.Appearance()
Appearance261.DEF = "circle1_Appearance"
Material262 = x3d.Material()
Material262.DEF = "circle1_Material"
Material262.diffuseColor = [0.9,0,0.7]
Material262.emissiveColor = [0.424956,0.483976,1]

Appearance261.material = Material262

Shape260.appearance = Appearance261
IndexedLineSet263 = x3d.IndexedLineSet()
IndexedLineSet263.USE = "Orbit1"

Shape260.geometry = IndexedLineSet263

Transform259.children.append(Shape260)

Group250.children.append(Transform259)
Transform264 = x3d.Transform()
Transform264.DEF = "Circle2"
Transform264.scale = [0.25,1,0.25]
Shape265 = x3d.Shape()
Shape265.DEF = "circle2_Shape"
Appearance266 = x3d.Appearance()
Appearance266.DEF = "circle2_Appearance"
Material267 = x3d.Material()
Material267.DEF = "circle2_Material"
Material267.diffuseColor = [0.9,0,0.7]
Material267.emissiveColor = [0.424956,0.483976,1]

Appearance266.material = Material267

Shape265.appearance = Appearance266
IndexedLineSet268 = x3d.IndexedLineSet()
IndexedLineSet268.USE = "Orbit1"

Shape265.geometry = IndexedLineSet268

Transform264.children.append(Shape265)

Group250.children.append(Transform264)

Scene13.children.append(Group250)
ROUTE269 = x3d.ROUTE()
ROUTE269.fromNode = "KickTimer"
ROUTE269.fromField = "fraction_changed"
ROUTE269.toNode = "HumanoidRootRotInterp"
ROUTE269.toField = "set_fraction"

Scene13.children.append(ROUTE269)
ROUTE270 = x3d.ROUTE()
ROUTE270.fromNode = "KickTimer"
ROUTE270.fromField = "fraction_changed"
ROUTE270.toNode = "HumanoidRootTransInterp"
ROUTE270.toField = "set_fraction"

Scene13.children.append(ROUTE270)
ROUTE271 = x3d.ROUTE()
ROUTE271.fromNode = "KickTimer"
ROUTE271.fromField = "fraction_changed"
ROUTE271.toNode = "sacroiliacRotInterp"
ROUTE271.toField = "set_fraction"

Scene13.children.append(ROUTE271)
ROUTE272 = x3d.ROUTE()
ROUTE272.fromNode = "KickTimer"
ROUTE272.fromField = "fraction_changed"
ROUTE272.toNode = "l_hipRotInterp"
ROUTE272.toField = "set_fraction"

Scene13.children.append(ROUTE272)
ROUTE273 = x3d.ROUTE()
ROUTE273.fromNode = "KickTimer"
ROUTE273.fromField = "fraction_changed"
ROUTE273.toNode = "l_kneeRotInterp"
ROUTE273.toField = "set_fraction"

Scene13.children.append(ROUTE273)
ROUTE274 = x3d.ROUTE()
ROUTE274.fromNode = "KickTimer"
ROUTE274.fromField = "fraction_changed"
ROUTE274.toNode = "l_ankleRotInterp"
ROUTE274.toField = "set_fraction"

Scene13.children.append(ROUTE274)
ROUTE275 = x3d.ROUTE()
ROUTE275.fromNode = "KickTimer"
ROUTE275.fromField = "fraction_changed"
ROUTE275.toNode = "l_subtalarRotInterp"
ROUTE275.toField = "set_fraction"

Scene13.children.append(ROUTE275)
ROUTE276 = x3d.ROUTE()
ROUTE276.fromNode = "KickTimer"
ROUTE276.fromField = "fraction_changed"
ROUTE276.toNode = "l_midtarsalRotInterp"
ROUTE276.toField = "set_fraction"

Scene13.children.append(ROUTE276)
ROUTE277 = x3d.ROUTE()
ROUTE277.fromNode = "KickTimer"
ROUTE277.fromField = "fraction_changed"
ROUTE277.toNode = "l_metatarsalRotInterp"
ROUTE277.toField = "set_fraction"

Scene13.children.append(ROUTE277)
ROUTE278 = x3d.ROUTE()
ROUTE278.fromNode = "KickTimer"
ROUTE278.fromField = "fraction_changed"
ROUTE278.toNode = "r_hipRotInterp"
ROUTE278.toField = "set_fraction"

Scene13.children.append(ROUTE278)
ROUTE279 = x3d.ROUTE()
ROUTE279.fromNode = "KickTimer"
ROUTE279.fromField = "fraction_changed"
ROUTE279.toNode = "r_kneeRotInterp"
ROUTE279.toField = "set_fraction"

Scene13.children.append(ROUTE279)
ROUTE280 = x3d.ROUTE()
ROUTE280.fromNode = "KickTimer"
ROUTE280.fromField = "fraction_changed"
ROUTE280.toNode = "r_ankleRotInterp"
ROUTE280.toField = "set_fraction"

Scene13.children.append(ROUTE280)
ROUTE281 = x3d.ROUTE()
ROUTE281.fromNode = "KickTimer"
ROUTE281.fromField = "fraction_changed"
ROUTE281.toNode = "r_subtalarRotInterp"
ROUTE281.toField = "set_fraction"

Scene13.children.append(ROUTE281)
ROUTE282 = x3d.ROUTE()
ROUTE282.fromNode = "KickTimer"
ROUTE282.fromField = "fraction_changed"
ROUTE282.toNode = "r_midtarsalRotInterp"
ROUTE282.toField = "set_fraction"

Scene13.children.append(ROUTE282)
ROUTE283 = x3d.ROUTE()
ROUTE283.fromNode = "KickTimer"
ROUTE283.fromField = "fraction_changed"
ROUTE283.toNode = "r_metatarsalRotInterp"
ROUTE283.toField = "set_fraction"

Scene13.children.append(ROUTE283)
ROUTE284 = x3d.ROUTE()
ROUTE284.fromNode = "KickTimer"
ROUTE284.fromField = "fraction_changed"
ROUTE284.toNode = "vl5RotInterp"
ROUTE284.toField = "set_fraction"

Scene13.children.append(ROUTE284)
ROUTE285 = x3d.ROUTE()
ROUTE285.fromNode = "KickTimer"
ROUTE285.fromField = "fraction_changed"
ROUTE285.toNode = "vl4RotInterp"
ROUTE285.toField = "set_fraction"

Scene13.children.append(ROUTE285)
ROUTE286 = x3d.ROUTE()
ROUTE286.fromNode = "KickTimer"
ROUTE286.fromField = "fraction_changed"
ROUTE286.toNode = "vl3RotInterp"
ROUTE286.toField = "set_fraction"

Scene13.children.append(ROUTE286)
ROUTE287 = x3d.ROUTE()
ROUTE287.fromNode = "KickTimer"
ROUTE287.fromField = "fraction_changed"
ROUTE287.toNode = "vl2RotInterp"
ROUTE287.toField = "set_fraction"

Scene13.children.append(ROUTE287)
ROUTE288 = x3d.ROUTE()
ROUTE288.fromNode = "KickTimer"
ROUTE288.fromField = "fraction_changed"
ROUTE288.toNode = "vl1RotInterp"
ROUTE288.toField = "set_fraction"

Scene13.children.append(ROUTE288)
ROUTE289 = x3d.ROUTE()
ROUTE289.fromNode = "KickTimer"
ROUTE289.fromField = "fraction_changed"
ROUTE289.toNode = "vt12RotInterp"
ROUTE289.toField = "set_fraction"

Scene13.children.append(ROUTE289)
ROUTE290 = x3d.ROUTE()
ROUTE290.fromNode = "KickTimer"
ROUTE290.fromField = "fraction_changed"
ROUTE290.toNode = "vt11RotInterp"
ROUTE290.toField = "set_fraction"

Scene13.children.append(ROUTE290)
ROUTE291 = x3d.ROUTE()
ROUTE291.fromNode = "KickTimer"
ROUTE291.fromField = "fraction_changed"
ROUTE291.toNode = "vt10RotInterp"
ROUTE291.toField = "set_fraction"

Scene13.children.append(ROUTE291)
ROUTE292 = x3d.ROUTE()
ROUTE292.fromNode = "KickTimer"
ROUTE292.fromField = "fraction_changed"
ROUTE292.toNode = "vt9RotInterp"
ROUTE292.toField = "set_fraction"

Scene13.children.append(ROUTE292)
ROUTE293 = x3d.ROUTE()
ROUTE293.fromNode = "KickTimer"
ROUTE293.fromField = "fraction_changed"
ROUTE293.toNode = "vt8RotInterp"
ROUTE293.toField = "set_fraction"

Scene13.children.append(ROUTE293)
ROUTE294 = x3d.ROUTE()
ROUTE294.fromNode = "KickTimer"
ROUTE294.fromField = "fraction_changed"
ROUTE294.toNode = "vt7RotInterp"
ROUTE294.toField = "set_fraction"

Scene13.children.append(ROUTE294)
ROUTE295 = x3d.ROUTE()
ROUTE295.fromNode = "KickTimer"
ROUTE295.fromField = "fraction_changed"
ROUTE295.toNode = "vt6RotInterp"
ROUTE295.toField = "set_fraction"

Scene13.children.append(ROUTE295)
ROUTE296 = x3d.ROUTE()
ROUTE296.fromNode = "KickTimer"
ROUTE296.fromField = "fraction_changed"
ROUTE296.toNode = "vt5RotInterp"
ROUTE296.toField = "set_fraction"

Scene13.children.append(ROUTE296)
ROUTE297 = x3d.ROUTE()
ROUTE297.fromNode = "KickTimer"
ROUTE297.fromField = "fraction_changed"
ROUTE297.toNode = "vt4RotInterp"
ROUTE297.toField = "set_fraction"

Scene13.children.append(ROUTE297)
ROUTE298 = x3d.ROUTE()
ROUTE298.fromNode = "KickTimer"
ROUTE298.fromField = "fraction_changed"
ROUTE298.toNode = "vt3RotInterp"
ROUTE298.toField = "set_fraction"

Scene13.children.append(ROUTE298)
ROUTE299 = x3d.ROUTE()
ROUTE299.fromNode = "KickTimer"
ROUTE299.fromField = "fraction_changed"
ROUTE299.toNode = "vt2RotInterp"
ROUTE299.toField = "set_fraction"

Scene13.children.append(ROUTE299)
ROUTE300 = x3d.ROUTE()
ROUTE300.fromNode = "KickTimer"
ROUTE300.fromField = "fraction_changed"
ROUTE300.toNode = "vt1RotInterp"
ROUTE300.toField = "set_fraction"

Scene13.children.append(ROUTE300)
ROUTE301 = x3d.ROUTE()
ROUTE301.fromNode = "KickTimer"
ROUTE301.fromField = "fraction_changed"
ROUTE301.toNode = "vc7RotInterp"
ROUTE301.toField = "set_fraction"

Scene13.children.append(ROUTE301)
ROUTE302 = x3d.ROUTE()
ROUTE302.fromNode = "KickTimer"
ROUTE302.fromField = "fraction_changed"
ROUTE302.toNode = "vc6RotInterp"
ROUTE302.toField = "set_fraction"

Scene13.children.append(ROUTE302)
ROUTE303 = x3d.ROUTE()
ROUTE303.fromNode = "KickTimer"
ROUTE303.fromField = "fraction_changed"
ROUTE303.toNode = "vc5RotInterp"
ROUTE303.toField = "set_fraction"

Scene13.children.append(ROUTE303)
ROUTE304 = x3d.ROUTE()
ROUTE304.fromNode = "KickTimer"
ROUTE304.fromField = "fraction_changed"
ROUTE304.toNode = "vc4RotInterp"
ROUTE304.toField = "set_fraction"

Scene13.children.append(ROUTE304)
ROUTE305 = x3d.ROUTE()
ROUTE305.fromNode = "KickTimer"
ROUTE305.fromField = "fraction_changed"
ROUTE305.toNode = "vc3RotInterp"
ROUTE305.toField = "set_fraction"

Scene13.children.append(ROUTE305)
ROUTE306 = x3d.ROUTE()
ROUTE306.fromNode = "KickTimer"
ROUTE306.fromField = "fraction_changed"
ROUTE306.toNode = "vc2RotInterp"
ROUTE306.toField = "set_fraction"

Scene13.children.append(ROUTE306)
ROUTE307 = x3d.ROUTE()
ROUTE307.fromNode = "KickTimer"
ROUTE307.fromField = "fraction_changed"
ROUTE307.toNode = "vc1RotInterp"
ROUTE307.toField = "set_fraction"

Scene13.children.append(ROUTE307)
ROUTE308 = x3d.ROUTE()
ROUTE308.fromNode = "KickTimer"
ROUTE308.fromField = "fraction_changed"
ROUTE308.toNode = "skullbaseRotInterp"
ROUTE308.toField = "set_fraction"

Scene13.children.append(ROUTE308)
ROUTE309 = x3d.ROUTE()
ROUTE309.fromNode = "KickTimer"
ROUTE309.fromField = "fraction_changed"
ROUTE309.toNode = "l_eyelid_jointRotInterp"
ROUTE309.toField = "set_fraction"

Scene13.children.append(ROUTE309)
ROUTE310 = x3d.ROUTE()
ROUTE310.fromNode = "KickTimer"
ROUTE310.fromField = "fraction_changed"
ROUTE310.toNode = "l_eyeball_jointRotInterp"
ROUTE310.toField = "set_fraction"

Scene13.children.append(ROUTE310)
ROUTE311 = x3d.ROUTE()
ROUTE311.fromNode = "KickTimer"
ROUTE311.fromField = "fraction_changed"
ROUTE311.toNode = "l_eyebrow_jointRotInterp"
ROUTE311.toField = "set_fraction"

Scene13.children.append(ROUTE311)
ROUTE312 = x3d.ROUTE()
ROUTE312.fromNode = "KickTimer"
ROUTE312.fromField = "fraction_changed"
ROUTE312.toNode = "r_eyelid_jointRotInterp"
ROUTE312.toField = "set_fraction"

Scene13.children.append(ROUTE312)
ROUTE313 = x3d.ROUTE()
ROUTE313.fromNode = "KickTimer"
ROUTE313.fromField = "fraction_changed"
ROUTE313.toNode = "r_eyeball_jointRotInterp"
ROUTE313.toField = "set_fraction"

Scene13.children.append(ROUTE313)
ROUTE314 = x3d.ROUTE()
ROUTE314.fromNode = "KickTimer"
ROUTE314.fromField = "fraction_changed"
ROUTE314.toNode = "r_eyebrow_jointRotInterp"
ROUTE314.toField = "set_fraction"

Scene13.children.append(ROUTE314)
ROUTE315 = x3d.ROUTE()
ROUTE315.fromNode = "KickTimer"
ROUTE315.fromField = "fraction_changed"
ROUTE315.toNode = "temporomandibularRotInterp"
ROUTE315.toField = "set_fraction"

Scene13.children.append(ROUTE315)
ROUTE316 = x3d.ROUTE()
ROUTE316.fromNode = "KickTimer"
ROUTE316.fromField = "fraction_changed"
ROUTE316.toNode = "l_sternoclavicularRotInterp"
ROUTE316.toField = "set_fraction"

Scene13.children.append(ROUTE316)
ROUTE317 = x3d.ROUTE()
ROUTE317.fromNode = "KickTimer"
ROUTE317.fromField = "fraction_changed"
ROUTE317.toNode = "l_acromioclavicularRotInterp"
ROUTE317.toField = "set_fraction"

Scene13.children.append(ROUTE317)
ROUTE318 = x3d.ROUTE()
ROUTE318.fromNode = "KickTimer"
ROUTE318.fromField = "fraction_changed"
ROUTE318.toNode = "l_shoulderRotInterp"
ROUTE318.toField = "set_fraction"

Scene13.children.append(ROUTE318)
ROUTE319 = x3d.ROUTE()
ROUTE319.fromNode = "KickTimer"
ROUTE319.fromField = "fraction_changed"
ROUTE319.toNode = "l_elbowRotInterp"
ROUTE319.toField = "set_fraction"

Scene13.children.append(ROUTE319)
ROUTE320 = x3d.ROUTE()
ROUTE320.fromNode = "KickTimer"
ROUTE320.fromField = "fraction_changed"
ROUTE320.toNode = "l_wristRotInterp"
ROUTE320.toField = "set_fraction"

Scene13.children.append(ROUTE320)
ROUTE321 = x3d.ROUTE()
ROUTE321.fromNode = "KickTimer"
ROUTE321.fromField = "fraction_changed"
ROUTE321.toNode = "l_thumb1RotInterp"
ROUTE321.toField = "set_fraction"

Scene13.children.append(ROUTE321)
ROUTE322 = x3d.ROUTE()
ROUTE322.fromNode = "KickTimer"
ROUTE322.fromField = "fraction_changed"
ROUTE322.toNode = "l_thumb2RotInterp"
ROUTE322.toField = "set_fraction"

Scene13.children.append(ROUTE322)
ROUTE323 = x3d.ROUTE()
ROUTE323.fromNode = "KickTimer"
ROUTE323.fromField = "fraction_changed"
ROUTE323.toNode = "l_thumb3RotInterp"
ROUTE323.toField = "set_fraction"

Scene13.children.append(ROUTE323)
ROUTE324 = x3d.ROUTE()
ROUTE324.fromNode = "KickTimer"
ROUTE324.fromField = "fraction_changed"
ROUTE324.toNode = "l_index0RotInterp"
ROUTE324.toField = "set_fraction"

Scene13.children.append(ROUTE324)
ROUTE325 = x3d.ROUTE()
ROUTE325.fromNode = "KickTimer"
ROUTE325.fromField = "fraction_changed"
ROUTE325.toNode = "l_index1RotInterp"
ROUTE325.toField = "set_fraction"

Scene13.children.append(ROUTE325)
ROUTE326 = x3d.ROUTE()
ROUTE326.fromNode = "KickTimer"
ROUTE326.fromField = "fraction_changed"
ROUTE326.toNode = "l_index2RotInterp"
ROUTE326.toField = "set_fraction"

Scene13.children.append(ROUTE326)
ROUTE327 = x3d.ROUTE()
ROUTE327.fromNode = "KickTimer"
ROUTE327.fromField = "fraction_changed"
ROUTE327.toNode = "l_index3RotInterp"
ROUTE327.toField = "set_fraction"

Scene13.children.append(ROUTE327)
ROUTE328 = x3d.ROUTE()
ROUTE328.fromNode = "KickTimer"
ROUTE328.fromField = "fraction_changed"
ROUTE328.toNode = "l_middle0RotInterp"
ROUTE328.toField = "set_fraction"

Scene13.children.append(ROUTE328)
ROUTE329 = x3d.ROUTE()
ROUTE329.fromNode = "KickTimer"
ROUTE329.fromField = "fraction_changed"
ROUTE329.toNode = "l_middle1RotInterp"
ROUTE329.toField = "set_fraction"

Scene13.children.append(ROUTE329)
ROUTE330 = x3d.ROUTE()
ROUTE330.fromNode = "KickTimer"
ROUTE330.fromField = "fraction_changed"
ROUTE330.toNode = "l_middle2RotInterp"
ROUTE330.toField = "set_fraction"

Scene13.children.append(ROUTE330)
ROUTE331 = x3d.ROUTE()
ROUTE331.fromNode = "KickTimer"
ROUTE331.fromField = "fraction_changed"
ROUTE331.toNode = "l_middle3RotInterp"
ROUTE331.toField = "set_fraction"

Scene13.children.append(ROUTE331)
ROUTE332 = x3d.ROUTE()
ROUTE332.fromNode = "KickTimer"
ROUTE332.fromField = "fraction_changed"
ROUTE332.toNode = "l_ring0RotInterp"
ROUTE332.toField = "set_fraction"

Scene13.children.append(ROUTE332)
ROUTE333 = x3d.ROUTE()
ROUTE333.fromNode = "KickTimer"
ROUTE333.fromField = "fraction_changed"
ROUTE333.toNode = "l_ring1RotInterp"
ROUTE333.toField = "set_fraction"

Scene13.children.append(ROUTE333)
ROUTE334 = x3d.ROUTE()
ROUTE334.fromNode = "KickTimer"
ROUTE334.fromField = "fraction_changed"
ROUTE334.toNode = "l_ring2RotInterp"
ROUTE334.toField = "set_fraction"

Scene13.children.append(ROUTE334)
ROUTE335 = x3d.ROUTE()
ROUTE335.fromNode = "KickTimer"
ROUTE335.fromField = "fraction_changed"
ROUTE335.toNode = "l_ring3RotInterp"
ROUTE335.toField = "set_fraction"

Scene13.children.append(ROUTE335)
ROUTE336 = x3d.ROUTE()
ROUTE336.fromNode = "KickTimer"
ROUTE336.fromField = "fraction_changed"
ROUTE336.toNode = "l_pinky0RotInterp"
ROUTE336.toField = "set_fraction"

Scene13.children.append(ROUTE336)
ROUTE337 = x3d.ROUTE()
ROUTE337.fromNode = "KickTimer"
ROUTE337.fromField = "fraction_changed"
ROUTE337.toNode = "l_pinky1RotInterp"
ROUTE337.toField = "set_fraction"

Scene13.children.append(ROUTE337)
ROUTE338 = x3d.ROUTE()
ROUTE338.fromNode = "KickTimer"
ROUTE338.fromField = "fraction_changed"
ROUTE338.toNode = "l_pinky2RotInterp"
ROUTE338.toField = "set_fraction"

Scene13.children.append(ROUTE338)
ROUTE339 = x3d.ROUTE()
ROUTE339.fromNode = "KickTimer"
ROUTE339.fromField = "fraction_changed"
ROUTE339.toNode = "l_pinky3RotInterp"
ROUTE339.toField = "set_fraction"

Scene13.children.append(ROUTE339)
ROUTE340 = x3d.ROUTE()
ROUTE340.fromNode = "KickTimer"
ROUTE340.fromField = "fraction_changed"
ROUTE340.toNode = "r_sternoclavicularRotInterp"
ROUTE340.toField = "set_fraction"

Scene13.children.append(ROUTE340)
ROUTE341 = x3d.ROUTE()
ROUTE341.fromNode = "KickTimer"
ROUTE341.fromField = "fraction_changed"
ROUTE341.toNode = "r_acromioclavicularRotInterp"
ROUTE341.toField = "set_fraction"

Scene13.children.append(ROUTE341)
ROUTE342 = x3d.ROUTE()
ROUTE342.fromNode = "KickTimer"
ROUTE342.fromField = "fraction_changed"
ROUTE342.toNode = "r_shoulderRotInterp"
ROUTE342.toField = "set_fraction"

Scene13.children.append(ROUTE342)
ROUTE343 = x3d.ROUTE()
ROUTE343.fromNode = "KickTimer"
ROUTE343.fromField = "fraction_changed"
ROUTE343.toNode = "r_elbowRotInterp"
ROUTE343.toField = "set_fraction"

Scene13.children.append(ROUTE343)
ROUTE344 = x3d.ROUTE()
ROUTE344.fromNode = "KickTimer"
ROUTE344.fromField = "fraction_changed"
ROUTE344.toNode = "r_wristRotInterp"
ROUTE344.toField = "set_fraction"

Scene13.children.append(ROUTE344)
ROUTE345 = x3d.ROUTE()
ROUTE345.fromNode = "KickTimer"
ROUTE345.fromField = "fraction_changed"
ROUTE345.toNode = "r_thumb1RotInterp"
ROUTE345.toField = "set_fraction"

Scene13.children.append(ROUTE345)
ROUTE346 = x3d.ROUTE()
ROUTE346.fromNode = "KickTimer"
ROUTE346.fromField = "fraction_changed"
ROUTE346.toNode = "r_thumb2RotInterp"
ROUTE346.toField = "set_fraction"

Scene13.children.append(ROUTE346)
ROUTE347 = x3d.ROUTE()
ROUTE347.fromNode = "KickTimer"
ROUTE347.fromField = "fraction_changed"
ROUTE347.toNode = "r_thumb3RotInterp"
ROUTE347.toField = "set_fraction"

Scene13.children.append(ROUTE347)
ROUTE348 = x3d.ROUTE()
ROUTE348.fromNode = "KickTimer"
ROUTE348.fromField = "fraction_changed"
ROUTE348.toNode = "r_index0RotInterp"
ROUTE348.toField = "set_fraction"

Scene13.children.append(ROUTE348)
ROUTE349 = x3d.ROUTE()
ROUTE349.fromNode = "KickTimer"
ROUTE349.fromField = "fraction_changed"
ROUTE349.toNode = "r_index1RotInterp"
ROUTE349.toField = "set_fraction"

Scene13.children.append(ROUTE349)
ROUTE350 = x3d.ROUTE()
ROUTE350.fromNode = "KickTimer"
ROUTE350.fromField = "fraction_changed"
ROUTE350.toNode = "r_index2RotInterp"
ROUTE350.toField = "set_fraction"

Scene13.children.append(ROUTE350)
ROUTE351 = x3d.ROUTE()
ROUTE351.fromNode = "KickTimer"
ROUTE351.fromField = "fraction_changed"
ROUTE351.toNode = "r_index3RotInterp"
ROUTE351.toField = "set_fraction"

Scene13.children.append(ROUTE351)
ROUTE352 = x3d.ROUTE()
ROUTE352.fromNode = "KickTimer"
ROUTE352.fromField = "fraction_changed"
ROUTE352.toNode = "r_middle0RotInterp"
ROUTE352.toField = "set_fraction"

Scene13.children.append(ROUTE352)
ROUTE353 = x3d.ROUTE()
ROUTE353.fromNode = "KickTimer"
ROUTE353.fromField = "fraction_changed"
ROUTE353.toNode = "r_middle1RotInterp"
ROUTE353.toField = "set_fraction"

Scene13.children.append(ROUTE353)
ROUTE354 = x3d.ROUTE()
ROUTE354.fromNode = "KickTimer"
ROUTE354.fromField = "fraction_changed"
ROUTE354.toNode = "r_middle2RotInterp"
ROUTE354.toField = "set_fraction"

Scene13.children.append(ROUTE354)
ROUTE355 = x3d.ROUTE()
ROUTE355.fromNode = "KickTimer"
ROUTE355.fromField = "fraction_changed"
ROUTE355.toNode = "r_middle3RotInterp"
ROUTE355.toField = "set_fraction"

Scene13.children.append(ROUTE355)
ROUTE356 = x3d.ROUTE()
ROUTE356.fromNode = "KickTimer"
ROUTE356.fromField = "fraction_changed"
ROUTE356.toNode = "r_ring0RotInterp"
ROUTE356.toField = "set_fraction"

Scene13.children.append(ROUTE356)
ROUTE357 = x3d.ROUTE()
ROUTE357.fromNode = "KickTimer"
ROUTE357.fromField = "fraction_changed"
ROUTE357.toNode = "r_ring1RotInterp"
ROUTE357.toField = "set_fraction"

Scene13.children.append(ROUTE357)
ROUTE358 = x3d.ROUTE()
ROUTE358.fromNode = "KickTimer"
ROUTE358.fromField = "fraction_changed"
ROUTE358.toNode = "r_ring2RotInterp"
ROUTE358.toField = "set_fraction"

Scene13.children.append(ROUTE358)
ROUTE359 = x3d.ROUTE()
ROUTE359.fromNode = "KickTimer"
ROUTE359.fromField = "fraction_changed"
ROUTE359.toNode = "r_ring3RotInterp"
ROUTE359.toField = "set_fraction"

Scene13.children.append(ROUTE359)
ROUTE360 = x3d.ROUTE()
ROUTE360.fromNode = "KickTimer"
ROUTE360.fromField = "fraction_changed"
ROUTE360.toNode = "r_pinky0RotInterp"
ROUTE360.toField = "set_fraction"

Scene13.children.append(ROUTE360)
ROUTE361 = x3d.ROUTE()
ROUTE361.fromNode = "KickTimer"
ROUTE361.fromField = "fraction_changed"
ROUTE361.toNode = "r_pinky1RotInterp"
ROUTE361.toField = "set_fraction"

Scene13.children.append(ROUTE361)
ROUTE362 = x3d.ROUTE()
ROUTE362.fromNode = "KickTimer"
ROUTE362.fromField = "fraction_changed"
ROUTE362.toNode = "r_pinky2RotInterp"
ROUTE362.toField = "set_fraction"

Scene13.children.append(ROUTE362)
ROUTE363 = x3d.ROUTE()
ROUTE363.fromNode = "KickTimer"
ROUTE363.fromField = "fraction_changed"
ROUTE363.toNode = "r_pinky3RotInterp"
ROUTE363.toField = "set_fraction"

Scene13.children.append(ROUTE363)
ROUTE364 = x3d.ROUTE()
ROUTE364.fromNode = "HumanoidRootRotInterp"
ROUTE364.fromField = "value_changed"
ROUTE364.toNode = "Joe_HumanoidRoot"
ROUTE364.toField = "set_rotation"

Scene13.children.append(ROUTE364)
ROUTE365 = x3d.ROUTE()
ROUTE365.fromNode = "HumanoidRootTransInterp"
ROUTE365.fromField = "value_changed"
ROUTE365.toNode = "Joe_HumanoidRoot"
ROUTE365.toField = "set_translation"

Scene13.children.append(ROUTE365)
ROUTE366 = x3d.ROUTE()
ROUTE366.fromNode = "sacroiliacRotInterp"
ROUTE366.fromField = "value_changed"
ROUTE366.toNode = "Joe_sacroiliac"
ROUTE366.toField = "set_rotation"

Scene13.children.append(ROUTE366)
ROUTE367 = x3d.ROUTE()
ROUTE367.fromNode = "l_hipRotInterp"
ROUTE367.fromField = "value_changed"
ROUTE367.toNode = "Joe_l_hip"
ROUTE367.toField = "set_rotation"

Scene13.children.append(ROUTE367)
ROUTE368 = x3d.ROUTE()
ROUTE368.fromNode = "l_kneeRotInterp"
ROUTE368.fromField = "value_changed"
ROUTE368.toNode = "Joe_l_knee"
ROUTE368.toField = "set_rotation"

Scene13.children.append(ROUTE368)
ROUTE369 = x3d.ROUTE()
ROUTE369.fromNode = "l_ankleRotInterp"
ROUTE369.fromField = "value_changed"
ROUTE369.toNode = "Joe_l_ankle"
ROUTE369.toField = "set_rotation"

Scene13.children.append(ROUTE369)
ROUTE370 = x3d.ROUTE()
ROUTE370.fromNode = "l_subtalarRotInterp"
ROUTE370.fromField = "value_changed"
ROUTE370.toNode = "Joe_l_subtalar"
ROUTE370.toField = "set_rotation"

Scene13.children.append(ROUTE370)
ROUTE371 = x3d.ROUTE()
ROUTE371.fromNode = "l_midtarsalRotInterp"
ROUTE371.fromField = "value_changed"
ROUTE371.toNode = "Joe_l_midtarsal"
ROUTE371.toField = "set_rotation"

Scene13.children.append(ROUTE371)
ROUTE372 = x3d.ROUTE()
ROUTE372.fromNode = "l_metatarsalRotInterp"
ROUTE372.fromField = "value_changed"
ROUTE372.toNode = "Joe_l_metatarsal"
ROUTE372.toField = "set_rotation"

Scene13.children.append(ROUTE372)
ROUTE373 = x3d.ROUTE()
ROUTE373.fromNode = "r_hipRotInterp"
ROUTE373.fromField = "value_changed"
ROUTE373.toNode = "Joe_r_hip"
ROUTE373.toField = "set_rotation"

Scene13.children.append(ROUTE373)
ROUTE374 = x3d.ROUTE()
ROUTE374.fromNode = "r_kneeRotInterp"
ROUTE374.fromField = "value_changed"
ROUTE374.toNode = "Joe_r_knee"
ROUTE374.toField = "set_rotation"

Scene13.children.append(ROUTE374)
ROUTE375 = x3d.ROUTE()
ROUTE375.fromNode = "r_ankleRotInterp"
ROUTE375.fromField = "value_changed"
ROUTE375.toNode = "Joe_r_ankle"
ROUTE375.toField = "set_rotation"

Scene13.children.append(ROUTE375)
ROUTE376 = x3d.ROUTE()
ROUTE376.fromNode = "r_subtalarRotInterp"
ROUTE376.fromField = "value_changed"
ROUTE376.toNode = "Joe_r_subtalar"
ROUTE376.toField = "set_rotation"

Scene13.children.append(ROUTE376)
ROUTE377 = x3d.ROUTE()
ROUTE377.fromNode = "r_midtarsalRotInterp"
ROUTE377.fromField = "value_changed"
ROUTE377.toNode = "Joe_r_midtarsal"
ROUTE377.toField = "set_rotation"

Scene13.children.append(ROUTE377)
ROUTE378 = x3d.ROUTE()
ROUTE378.fromNode = "r_metatarsalRotInterp"
ROUTE378.fromField = "value_changed"
ROUTE378.toNode = "Joe_r_metatarsal"
ROUTE378.toField = "set_rotation"

Scene13.children.append(ROUTE378)
ROUTE379 = x3d.ROUTE()
ROUTE379.fromNode = "vl5RotInterp"
ROUTE379.fromField = "value_changed"
ROUTE379.toNode = "Joe_vl5"
ROUTE379.toField = "set_rotation"

Scene13.children.append(ROUTE379)
ROUTE380 = x3d.ROUTE()
ROUTE380.fromNode = "vl4RotInterp"
ROUTE380.fromField = "value_changed"
ROUTE380.toNode = "Joe_vl4"
ROUTE380.toField = "set_rotation"

Scene13.children.append(ROUTE380)
ROUTE381 = x3d.ROUTE()
ROUTE381.fromNode = "vl3RotInterp"
ROUTE381.fromField = "value_changed"
ROUTE381.toNode = "Joe_vl3"
ROUTE381.toField = "set_rotation"

Scene13.children.append(ROUTE381)
ROUTE382 = x3d.ROUTE()
ROUTE382.fromNode = "vl2RotInterp"
ROUTE382.fromField = "value_changed"
ROUTE382.toNode = "Joe_vl2"
ROUTE382.toField = "set_rotation"

Scene13.children.append(ROUTE382)
ROUTE383 = x3d.ROUTE()
ROUTE383.fromNode = "vl1RotInterp"
ROUTE383.fromField = "value_changed"
ROUTE383.toNode = "Joe_vl1"
ROUTE383.toField = "set_rotation"

Scene13.children.append(ROUTE383)
ROUTE384 = x3d.ROUTE()
ROUTE384.fromNode = "vt12RotInterp"
ROUTE384.fromField = "value_changed"
ROUTE384.toNode = "Joe_vt12"
ROUTE384.toField = "set_rotation"

Scene13.children.append(ROUTE384)
ROUTE385 = x3d.ROUTE()
ROUTE385.fromNode = "vt11RotInterp"
ROUTE385.fromField = "value_changed"
ROUTE385.toNode = "Joe_vt11"
ROUTE385.toField = "set_rotation"

Scene13.children.append(ROUTE385)
ROUTE386 = x3d.ROUTE()
ROUTE386.fromNode = "vt10RotInterp"
ROUTE386.fromField = "value_changed"
ROUTE386.toNode = "Joe_vt10"
ROUTE386.toField = "set_rotation"

Scene13.children.append(ROUTE386)
ROUTE387 = x3d.ROUTE()
ROUTE387.fromNode = "vt9RotInterp"
ROUTE387.fromField = "value_changed"
ROUTE387.toNode = "Joe_vt9"
ROUTE387.toField = "set_rotation"

Scene13.children.append(ROUTE387)
ROUTE388 = x3d.ROUTE()
ROUTE388.fromNode = "vt8RotInterp"
ROUTE388.fromField = "value_changed"
ROUTE388.toNode = "Joe_vt8"
ROUTE388.toField = "set_rotation"

Scene13.children.append(ROUTE388)
ROUTE389 = x3d.ROUTE()
ROUTE389.fromNode = "vt7RotInterp"
ROUTE389.fromField = "value_changed"
ROUTE389.toNode = "Joe_vt7"
ROUTE389.toField = "set_rotation"

Scene13.children.append(ROUTE389)
ROUTE390 = x3d.ROUTE()
ROUTE390.fromNode = "vt6RotInterp"
ROUTE390.fromField = "value_changed"
ROUTE390.toNode = "Joe_vt6"
ROUTE390.toField = "set_rotation"

Scene13.children.append(ROUTE390)
ROUTE391 = x3d.ROUTE()
ROUTE391.fromNode = "vt5RotInterp"
ROUTE391.fromField = "value_changed"
ROUTE391.toNode = "Joe_vt5"
ROUTE391.toField = "set_rotation"

Scene13.children.append(ROUTE391)
ROUTE392 = x3d.ROUTE()
ROUTE392.fromNode = "vt4RotInterp"
ROUTE392.fromField = "value_changed"
ROUTE392.toNode = "Joe_vt4"
ROUTE392.toField = "set_rotation"

Scene13.children.append(ROUTE392)
ROUTE393 = x3d.ROUTE()
ROUTE393.fromNode = "vt3RotInterp"
ROUTE393.fromField = "value_changed"
ROUTE393.toNode = "Joe_vt3"
ROUTE393.toField = "set_rotation"

Scene13.children.append(ROUTE393)
ROUTE394 = x3d.ROUTE()
ROUTE394.fromNode = "vt2RotInterp"
ROUTE394.fromField = "value_changed"
ROUTE394.toNode = "Joe_vt2"
ROUTE394.toField = "set_rotation"

Scene13.children.append(ROUTE394)
ROUTE395 = x3d.ROUTE()
ROUTE395.fromNode = "vt1RotInterp"
ROUTE395.fromField = "value_changed"
ROUTE395.toNode = "Joe_vt1"
ROUTE395.toField = "set_rotation"

Scene13.children.append(ROUTE395)
ROUTE396 = x3d.ROUTE()
ROUTE396.fromNode = "vc7RotInterp"
ROUTE396.fromField = "value_changed"
ROUTE396.toNode = "Joe_vc7"
ROUTE396.toField = "set_rotation"

Scene13.children.append(ROUTE396)
ROUTE397 = x3d.ROUTE()
ROUTE397.fromNode = "vc6RotInterp"
ROUTE397.fromField = "value_changed"
ROUTE397.toNode = "Joe_vc6"
ROUTE397.toField = "set_rotation"

Scene13.children.append(ROUTE397)
ROUTE398 = x3d.ROUTE()
ROUTE398.fromNode = "vc5RotInterp"
ROUTE398.fromField = "value_changed"
ROUTE398.toNode = "Joe_vc5"
ROUTE398.toField = "set_rotation"

Scene13.children.append(ROUTE398)
ROUTE399 = x3d.ROUTE()
ROUTE399.fromNode = "vc4RotInterp"
ROUTE399.fromField = "value_changed"
ROUTE399.toNode = "Joe_vc4"
ROUTE399.toField = "set_rotation"

Scene13.children.append(ROUTE399)
ROUTE400 = x3d.ROUTE()
ROUTE400.fromNode = "vc3RotInterp"
ROUTE400.fromField = "value_changed"
ROUTE400.toNode = "Joe_vc3"
ROUTE400.toField = "set_rotation"

Scene13.children.append(ROUTE400)
ROUTE401 = x3d.ROUTE()
ROUTE401.fromNode = "vc2RotInterp"
ROUTE401.fromField = "value_changed"
ROUTE401.toNode = "Joe_vc2"
ROUTE401.toField = "set_rotation"

Scene13.children.append(ROUTE401)
ROUTE402 = x3d.ROUTE()
ROUTE402.fromNode = "vc1RotInterp"
ROUTE402.fromField = "value_changed"
ROUTE402.toNode = "Joe_vc1"
ROUTE402.toField = "set_rotation"

Scene13.children.append(ROUTE402)
ROUTE403 = x3d.ROUTE()
ROUTE403.fromNode = "skullbaseRotInterp"
ROUTE403.fromField = "value_changed"
ROUTE403.toNode = "Joe_skullbase"
ROUTE403.toField = "set_rotation"

Scene13.children.append(ROUTE403)
ROUTE404 = x3d.ROUTE()
ROUTE404.fromNode = "l_eyelid_jointRotInterp"
ROUTE404.fromField = "value_changed"
ROUTE404.toNode = "Joe_l_eyelid_joint"
ROUTE404.toField = "set_rotation"

Scene13.children.append(ROUTE404)
ROUTE405 = x3d.ROUTE()
ROUTE405.fromNode = "l_eyeball_jointRotInterp"
ROUTE405.fromField = "value_changed"
ROUTE405.toNode = "Joe_l_eyeball_joint"
ROUTE405.toField = "set_rotation"

Scene13.children.append(ROUTE405)
ROUTE406 = x3d.ROUTE()
ROUTE406.fromNode = "l_eyebrow_jointRotInterp"
ROUTE406.fromField = "value_changed"
ROUTE406.toNode = "Joe_l_eyebrow_joint"
ROUTE406.toField = "set_rotation"

Scene13.children.append(ROUTE406)
ROUTE407 = x3d.ROUTE()
ROUTE407.fromNode = "r_eyelid_jointRotInterp"
ROUTE407.fromField = "value_changed"
ROUTE407.toNode = "Joe_r_eyelid_joint"
ROUTE407.toField = "set_rotation"

Scene13.children.append(ROUTE407)
ROUTE408 = x3d.ROUTE()
ROUTE408.fromNode = "r_eyeball_jointRotInterp"
ROUTE408.fromField = "value_changed"
ROUTE408.toNode = "Joe_r_eyeball_joint"
ROUTE408.toField = "set_rotation"

Scene13.children.append(ROUTE408)
ROUTE409 = x3d.ROUTE()
ROUTE409.fromNode = "r_eyebrow_jointRotInterp"
ROUTE409.fromField = "value_changed"
ROUTE409.toNode = "Joe_r_eyebrow_joint"
ROUTE409.toField = "set_rotation"

Scene13.children.append(ROUTE409)
ROUTE410 = x3d.ROUTE()
ROUTE410.fromNode = "temporomandibularRotInterp"
ROUTE410.fromField = "value_changed"
ROUTE410.toNode = "Joe_temporomandibular"
ROUTE410.toField = "set_rotation"

Scene13.children.append(ROUTE410)
ROUTE411 = x3d.ROUTE()
ROUTE411.fromNode = "l_sternoclavicularRotInterp"
ROUTE411.fromField = "value_changed"
ROUTE411.toNode = "Joe_l_sternoclavicular"
ROUTE411.toField = "set_rotation"

Scene13.children.append(ROUTE411)
ROUTE412 = x3d.ROUTE()
ROUTE412.fromNode = "l_acromioclavicularRotInterp"
ROUTE412.fromField = "value_changed"
ROUTE412.toNode = "Joe_l_acromioclavicular"
ROUTE412.toField = "set_rotation"

Scene13.children.append(ROUTE412)
ROUTE413 = x3d.ROUTE()
ROUTE413.fromNode = "l_shoulderRotInterp"
ROUTE413.fromField = "value_changed"
ROUTE413.toNode = "Joe_l_shoulder"
ROUTE413.toField = "set_rotation"

Scene13.children.append(ROUTE413)
ROUTE414 = x3d.ROUTE()
ROUTE414.fromNode = "l_elbowRotInterp"
ROUTE414.fromField = "value_changed"
ROUTE414.toNode = "Joe_l_elbow"
ROUTE414.toField = "set_rotation"

Scene13.children.append(ROUTE414)
ROUTE415 = x3d.ROUTE()
ROUTE415.fromNode = "l_wristRotInterp"
ROUTE415.fromField = "value_changed"
ROUTE415.toNode = "Joe_l_wrist"
ROUTE415.toField = "set_rotation"

Scene13.children.append(ROUTE415)
ROUTE416 = x3d.ROUTE()
ROUTE416.fromNode = "l_thumb1RotInterp"
ROUTE416.fromField = "value_changed"
ROUTE416.toNode = "Joe_l_thumb1"
ROUTE416.toField = "set_rotation"

Scene13.children.append(ROUTE416)
ROUTE417 = x3d.ROUTE()
ROUTE417.fromNode = "l_thumb2RotInterp"
ROUTE417.fromField = "value_changed"
ROUTE417.toNode = "Joe_l_thumb2"
ROUTE417.toField = "set_rotation"

Scene13.children.append(ROUTE417)
ROUTE418 = x3d.ROUTE()
ROUTE418.fromNode = "l_thumb3RotInterp"
ROUTE418.fromField = "value_changed"
ROUTE418.toNode = "Joe_l_thumb3"
ROUTE418.toField = "set_rotation"

Scene13.children.append(ROUTE418)
ROUTE419 = x3d.ROUTE()
ROUTE419.fromNode = "l_index0RotInterp"
ROUTE419.fromField = "value_changed"
ROUTE419.toNode = "Joe_l_index0"
ROUTE419.toField = "set_rotation"

Scene13.children.append(ROUTE419)
ROUTE420 = x3d.ROUTE()
ROUTE420.fromNode = "l_index1RotInterp"
ROUTE420.fromField = "value_changed"
ROUTE420.toNode = "Joe_l_index1"
ROUTE420.toField = "set_rotation"

Scene13.children.append(ROUTE420)
ROUTE421 = x3d.ROUTE()
ROUTE421.fromNode = "l_index2RotInterp"
ROUTE421.fromField = "value_changed"
ROUTE421.toNode = "Joe_l_index2"
ROUTE421.toField = "set_rotation"

Scene13.children.append(ROUTE421)
ROUTE422 = x3d.ROUTE()
ROUTE422.fromNode = "l_index3RotInterp"
ROUTE422.fromField = "value_changed"
ROUTE422.toNode = "Joe_l_index3"
ROUTE422.toField = "set_rotation"

Scene13.children.append(ROUTE422)
ROUTE423 = x3d.ROUTE()
ROUTE423.fromNode = "l_middle0RotInterp"
ROUTE423.fromField = "value_changed"
ROUTE423.toNode = "Joe_l_middle0"
ROUTE423.toField = "set_rotation"

Scene13.children.append(ROUTE423)
ROUTE424 = x3d.ROUTE()
ROUTE424.fromNode = "l_middle1RotInterp"
ROUTE424.fromField = "value_changed"
ROUTE424.toNode = "Joe_l_middle1"
ROUTE424.toField = "set_rotation"

Scene13.children.append(ROUTE424)
ROUTE425 = x3d.ROUTE()
ROUTE425.fromNode = "l_middle2RotInterp"
ROUTE425.fromField = "value_changed"
ROUTE425.toNode = "Joe_l_middle2"
ROUTE425.toField = "set_rotation"

Scene13.children.append(ROUTE425)
ROUTE426 = x3d.ROUTE()
ROUTE426.fromNode = "l_middle3RotInterp"
ROUTE426.fromField = "value_changed"
ROUTE426.toNode = "Joe_l_middle3"
ROUTE426.toField = "set_rotation"

Scene13.children.append(ROUTE426)
ROUTE427 = x3d.ROUTE()
ROUTE427.fromNode = "l_ring0RotInterp"
ROUTE427.fromField = "value_changed"
ROUTE427.toNode = "Joe_l_ring0"
ROUTE427.toField = "set_rotation"

Scene13.children.append(ROUTE427)
ROUTE428 = x3d.ROUTE()
ROUTE428.fromNode = "l_ring1RotInterp"
ROUTE428.fromField = "value_changed"
ROUTE428.toNode = "Joe_l_ring1"
ROUTE428.toField = "set_rotation"

Scene13.children.append(ROUTE428)
ROUTE429 = x3d.ROUTE()
ROUTE429.fromNode = "l_ring2RotInterp"
ROUTE429.fromField = "value_changed"
ROUTE429.toNode = "Joe_l_ring2"
ROUTE429.toField = "set_rotation"

Scene13.children.append(ROUTE429)
ROUTE430 = x3d.ROUTE()
ROUTE430.fromNode = "l_ring3RotInterp"
ROUTE430.fromField = "value_changed"
ROUTE430.toNode = "Joe_l_ring3"
ROUTE430.toField = "set_rotation"

Scene13.children.append(ROUTE430)
ROUTE431 = x3d.ROUTE()
ROUTE431.fromNode = "l_pinky0RotInterp"
ROUTE431.fromField = "value_changed"
ROUTE431.toNode = "Joe_l_pinky0"
ROUTE431.toField = "set_rotation"

Scene13.children.append(ROUTE431)
ROUTE432 = x3d.ROUTE()
ROUTE432.fromNode = "l_pinky1RotInterp"
ROUTE432.fromField = "value_changed"
ROUTE432.toNode = "Joe_l_pinky1"
ROUTE432.toField = "set_rotation"

Scene13.children.append(ROUTE432)
ROUTE433 = x3d.ROUTE()
ROUTE433.fromNode = "l_pinky2RotInterp"
ROUTE433.fromField = "value_changed"
ROUTE433.toNode = "Joe_l_pinky2"
ROUTE433.toField = "set_rotation"

Scene13.children.append(ROUTE433)
ROUTE434 = x3d.ROUTE()
ROUTE434.fromNode = "l_pinky3RotInterp"
ROUTE434.fromField = "value_changed"
ROUTE434.toNode = "Joe_l_pinky3"
ROUTE434.toField = "set_rotation"

Scene13.children.append(ROUTE434)
ROUTE435 = x3d.ROUTE()
ROUTE435.fromNode = "r_sternoclavicularRotInterp"
ROUTE435.fromField = "value_changed"
ROUTE435.toNode = "Joe_r_sternoclavicular"
ROUTE435.toField = "set_rotation"

Scene13.children.append(ROUTE435)
ROUTE436 = x3d.ROUTE()
ROUTE436.fromNode = "r_acromioclavicularRotInterp"
ROUTE436.fromField = "value_changed"
ROUTE436.toNode = "Joe_r_acromioclavicular"
ROUTE436.toField = "set_rotation"

Scene13.children.append(ROUTE436)
ROUTE437 = x3d.ROUTE()
ROUTE437.fromNode = "r_shoulderRotInterp"
ROUTE437.fromField = "value_changed"
ROUTE437.toNode = "Joe_r_shoulder"
ROUTE437.toField = "set_rotation"

Scene13.children.append(ROUTE437)
ROUTE438 = x3d.ROUTE()
ROUTE438.fromNode = "r_elbowRotInterp"
ROUTE438.fromField = "value_changed"
ROUTE438.toNode = "Joe_r_elbow"
ROUTE438.toField = "set_rotation"

Scene13.children.append(ROUTE438)
ROUTE439 = x3d.ROUTE()
ROUTE439.fromNode = "r_wristRotInterp"
ROUTE439.fromField = "value_changed"
ROUTE439.toNode = "Joe_r_wrist"
ROUTE439.toField = "set_rotation"

Scene13.children.append(ROUTE439)
ROUTE440 = x3d.ROUTE()
ROUTE440.fromNode = "r_thumb1RotInterp"
ROUTE440.fromField = "value_changed"
ROUTE440.toNode = "Joe_r_thumb1"
ROUTE440.toField = "set_rotation"

Scene13.children.append(ROUTE440)
ROUTE441 = x3d.ROUTE()
ROUTE441.fromNode = "r_thumb2RotInterp"
ROUTE441.fromField = "value_changed"
ROUTE441.toNode = "Joe_r_thumb2"
ROUTE441.toField = "set_rotation"

Scene13.children.append(ROUTE441)
ROUTE442 = x3d.ROUTE()
ROUTE442.fromNode = "r_thumb3RotInterp"
ROUTE442.fromField = "value_changed"
ROUTE442.toNode = "Joe_r_thumb3"
ROUTE442.toField = "set_rotation"

Scene13.children.append(ROUTE442)
ROUTE443 = x3d.ROUTE()
ROUTE443.fromNode = "r_index0RotInterp"
ROUTE443.fromField = "value_changed"
ROUTE443.toNode = "Joe_r_index0"
ROUTE443.toField = "set_rotation"

Scene13.children.append(ROUTE443)
ROUTE444 = x3d.ROUTE()
ROUTE444.fromNode = "r_index1RotInterp"
ROUTE444.fromField = "value_changed"
ROUTE444.toNode = "Joe_r_index1"
ROUTE444.toField = "set_rotation"

Scene13.children.append(ROUTE444)
ROUTE445 = x3d.ROUTE()
ROUTE445.fromNode = "r_index2RotInterp"
ROUTE445.fromField = "value_changed"
ROUTE445.toNode = "Joe_r_index2"
ROUTE445.toField = "set_rotation"

Scene13.children.append(ROUTE445)
ROUTE446 = x3d.ROUTE()
ROUTE446.fromNode = "r_index3RotInterp"
ROUTE446.fromField = "value_changed"
ROUTE446.toNode = "Joe_r_index3"
ROUTE446.toField = "set_rotation"

Scene13.children.append(ROUTE446)
ROUTE447 = x3d.ROUTE()
ROUTE447.fromNode = "r_middle0RotInterp"
ROUTE447.fromField = "value_changed"
ROUTE447.toNode = "Joe_r_middle0"
ROUTE447.toField = "set_rotation"

Scene13.children.append(ROUTE447)
ROUTE448 = x3d.ROUTE()
ROUTE448.fromNode = "r_middle1RotInterp"
ROUTE448.fromField = "value_changed"
ROUTE448.toNode = "Joe_r_middle1"
ROUTE448.toField = "set_rotation"

Scene13.children.append(ROUTE448)
ROUTE449 = x3d.ROUTE()
ROUTE449.fromNode = "r_middle2RotInterp"
ROUTE449.fromField = "value_changed"
ROUTE449.toNode = "Joe_r_middle2"
ROUTE449.toField = "set_rotation"

Scene13.children.append(ROUTE449)
ROUTE450 = x3d.ROUTE()
ROUTE450.fromNode = "r_middle3RotInterp"
ROUTE450.fromField = "value_changed"
ROUTE450.toNode = "Joe_r_middle3"
ROUTE450.toField = "set_rotation"

Scene13.children.append(ROUTE450)
ROUTE451 = x3d.ROUTE()
ROUTE451.fromNode = "r_ring0RotInterp"
ROUTE451.fromField = "value_changed"
ROUTE451.toNode = "Joe_r_ring0"
ROUTE451.toField = "set_rotation"

Scene13.children.append(ROUTE451)
ROUTE452 = x3d.ROUTE()
ROUTE452.fromNode = "r_ring1RotInterp"
ROUTE452.fromField = "value_changed"
ROUTE452.toNode = "Joe_r_ring1"
ROUTE452.toField = "set_rotation"

Scene13.children.append(ROUTE452)
ROUTE453 = x3d.ROUTE()
ROUTE453.fromNode = "r_ring2RotInterp"
ROUTE453.fromField = "value_changed"
ROUTE453.toNode = "Joe_r_ring2"
ROUTE453.toField = "set_rotation"

Scene13.children.append(ROUTE453)
ROUTE454 = x3d.ROUTE()
ROUTE454.fromNode = "r_ring3RotInterp"
ROUTE454.fromField = "value_changed"
ROUTE454.toNode = "Joe_r_ring3"
ROUTE454.toField = "set_rotation"

Scene13.children.append(ROUTE454)
ROUTE455 = x3d.ROUTE()
ROUTE455.fromNode = "r_pinky0RotInterp"
ROUTE455.fromField = "value_changed"
ROUTE455.toNode = "Joe_r_pinky0"
ROUTE455.toField = "set_rotation"

Scene13.children.append(ROUTE455)
ROUTE456 = x3d.ROUTE()
ROUTE456.fromNode = "r_pinky1RotInterp"
ROUTE456.fromField = "value_changed"
ROUTE456.toNode = "Joe_r_pinky1"
ROUTE456.toField = "set_rotation"

Scene13.children.append(ROUTE456)
ROUTE457 = x3d.ROUTE()
ROUTE457.fromNode = "r_pinky2RotInterp"
ROUTE457.fromField = "value_changed"
ROUTE457.toNode = "Joe_r_pinky2"
ROUTE457.toField = "set_rotation"

Scene13.children.append(ROUTE457)
ROUTE458 = x3d.ROUTE()
ROUTE458.fromNode = "r_pinky3RotInterp"
ROUTE458.fromField = "value_changed"
ROUTE458.toNode = "Joe_r_pinky3"
ROUTE458.toField = "set_rotation"

Scene13.children.append(ROUTE458)
ROUTE459 = x3d.ROUTE()
ROUTE459.fromNode = "KickTimer"
ROUTE459.fromField = "fraction_changed"
ROUTE459.toNode = "skull_tipTest"
ROUTE459.toField = "set_fraction"

Scene13.children.append(ROUTE459)
ROUTE460 = x3d.ROUTE()
ROUTE460.fromNode = "skull_tipTest"
ROUTE460.fromField = "value_changed"
ROUTE460.toNode = "Joe_skull_tipTest"
ROUTE460.toField = "set_weight"

Scene13.children.append(ROUTE460)
ROUTE461 = x3d.ROUTE()
ROUTE461.fromNode = "KickTimer"
ROUTE461.fromField = "fraction_changed"
ROUTE461.toNode = "skinTexTransTest"
ROUTE461.toField = "set_fraction"

Scene13.children.append(ROUTE461)
ROUTE462 = x3d.ROUTE()
ROUTE462.fromNode = "skinTexTransTest"
ROUTE462.fromField = "value_changed"
ROUTE462.toNode = "kicktextrans"
ROUTE462.toField = "set_rotation"

Scene13.children.append(ROUTE462)
ROUTE463 = x3d.ROUTE()
ROUTE463.fromNode = "KickTimer"
ROUTE463.fromField = "fraction_changed"
ROUTE463.toNode = "ballTransInterp"
ROUTE463.toField = "set_fraction"

Scene13.children.append(ROUTE463)
ROUTE464 = x3d.ROUTE()
ROUTE464.fromNode = "ballTransInterp"
ROUTE464.fromField = "value_changed"
ROUTE464.toNode = "SBall"
ROUTE464.toField = "set_translation"

Scene13.children.append(ROUTE464)
ROUTE465 = x3d.ROUTE()
ROUTE465.fromNode = "KickTimer"
ROUTE465.fromField = "fraction_changed"
ROUTE465.toNode = "ballRotInterp"
ROUTE465.toField = "set_fraction"

Scene13.children.append(ROUTE465)
ROUTE466 = x3d.ROUTE()
ROUTE466.fromNode = "ballRotInterp"
ROUTE466.fromField = "value_changed"
ROUTE466.toNode = "SBall"
ROUTE466.toField = "set_rotation"

Scene13.children.append(ROUTE466)

X3D0.Scene = Scene13
f = open("JoeSkinTexcoordDisplacerKickUpdate2.new.python.x3d", mode="w", encoding="utf-8")
f.write(X3D0.XML())
f.close()
f = open("JoeSkinTexcoordDisplacerKickUpdate2.new.python.x3dv", mode="w", encoding="utf-8")
f.write(X3D0.VRML())
f.close()
#f = open("JoeSkinTexcoordDisplacerKickUpdate2.new.python.json", mode="w", encoding="utf-8")
#f.write(X3D0.JSON())
#f.close()
