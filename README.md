# Drawing-bot  
Collaborative Robot application that drawing picture.

## Test Video
[![Video Label](http://img.youtube.com/vi/5INfNjucqbA/0.jpg)](https://www.youtube.com/watch?v=5INfNjucqbA)  

## Team
|Name|Team Role|
|:--:|:--:|
|[Sanghyun Park(Me)](https://github.com/SanghyunPark01)|Path planning, UI(Design & Interaction)|
|[Ilhyuk Kwon](https://github.com/kwon1h)|Team Leader|
|[Bumsoo Cho](https://github.com/retia0804)|Dart-Studio Programming(Robot System)|
|Euijung Yang|Hardware Design, Process Debugging|

## Application Overview

### 1) Hardware
* **Robot**  
  Doosan Robotics Collaborative Robot: M Series
* **Camera**  
  Any Webcam

### 2) System Algorithm
* Main Process  
  1\) Select Mode (Mouse Drawing, Camera)  
  2\) Drawing or Capture  
  3\) Path Planning(Our Algorithm)  
  4\) Get Path & Send to Robot
* Robot Process  
  1\) Make Thread (Get Path Thread, Drawing Thread)  
  2-1\) (Get Path Thread) Receive Path Data from Main Process & push to path queue  
  2-2\) (Drawing Thread) pop path queue & Move Robot  

### 3) More Info
See [Youtube Link](https://www.youtube.com/watch?v=5INfNjucqbA)  

## Contact  
E-Mail: pash0302@gmail.com  
E-Mail: pash0302@naver.com