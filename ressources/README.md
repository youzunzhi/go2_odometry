## Hardware description and building instructions
 todo: 
 config QTM




 # Parts
## Helmet
##### 

| ![](photos/image-1736521381091.png?raw=1) | ![](photos/image-1736521391096.png?raw=1) |
| :------------------------------------------------------------------------------------------------------------------------- | :------------------------------------------------------------------------------------------------------------------------- |

##### Files

| File name                 | Use                                     |
| :------------------------ | :-------------------------------------- |
| mocap_helmet_v2.FCStd     | CAD file, modify this one if needed     |
| MocapHelmet_v2_1.stl      | STL file                                |


##### Putting on

1. Install 8 motion capture round markers with 15mm diameter plastic base on the slots,  secure with double sided tape between the marker and the part.
2. Attach the helmet to the "head" of the Go2 with 4 M3x20mm screws.

##### Modifying

To modify this part first modify the body called "MocapHelmetGO2" then convert it to mesh and subtract the mesh called "TêteScaled001". You can then export the mesh as a STL for 3D printing.

## Rail attachement

| ![](photos/image-1736521515563.png?raw=1) | ![](photos/image-1736521527914.png?raw=1) |
| :------------------------------------------------------------------------------------------------------------------------- | :------------------------------------------------------------------------------------------------------------------------- |

##### 

##### Files

| File name             | Use                                     |
| :-------------------- | :-------------------------------------- |
| go2_mocap_rails.FCStd | CAD file, modify this one if needed     |
| go2_mocap_rails.stl   | STL file                                |


##### Putting on

1. Install 2 M2 nuts in hexagonal slots (see pic) and 2 M2x12mm screws from the other side
   1. If they don't fit try heating them with a heat gun but be careful to not deform the plastic or else the part won't slide in the rails anymore.

<img src="photos/image-1736521593508.png?raw=1" width="610" height="null" />

1. Install 2 motion capture round markers with 15mm diameter plastic base on the slots, secure with double sided tape between the marker and the part.
2. Slide the part in the rail from behind as far as you can and tighten the screws to secure.

##### Modifying

No special precautions

## Back or "Tail"

##### <img src="photos/image-1736522398812.png?raw=1" width="429" />

##### Files

| File name            | Use                                     |
| :------------------- | :-------------------------------------- |
| go2_mocap_back.FCStd | CAD file, modify this one if needed     |
| go2_mocap_back.stl   | STL file                                |


##### Putting on

1. Install 1 motion capture round markers with 15mm diameter plastic base on the slots,  secure with double sided tape between the marker and the part.
2. Use double sided tape on the hexagonal face and place the part in the slot located in the back of the robot as shown in the image below.

<img src="photos/image-1736522275053.png?raw=1" width="447" height="null" />

##### Modifying

No special precautions

# Motion Capture (Qualisys) Setup

The settings for the rigid body can be found in the file _Mocap settings/go2.xml_ and can be loaded in the qualisys software in the _setting panels_ -> _6DOF tracking _-> _Load Bodies_


