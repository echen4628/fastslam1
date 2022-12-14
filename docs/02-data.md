---
layout: page
title: Data
permalink: /data/
---

# Data Source
We are using the Multi-robot Cooperative Localization and Mapping [citation here] dataset collected by Leung et. al at the University of Toronto Institute for Aerospace Studies. The dataset consists of nine sub-datasets, which differ in experimental runtime and setup. Data was produced using 5 moving robots and 15 static landmarks.

<div>
  <img src="https://echen4628.github.io/fastslam1/assets/img/Robot_and_Landmarks.png" alt="Robots and Landmarks" width="100" />
</div>

##### Caption Here
 
# Data Description
In each sub-dataset, there are seventeen files:
<style type="text/css">
.tg  {border-collapse:collapse;border-spacing:0;}
.tg td{border-color:black;border-style:solid;border-width:1px;font-family:Arial, sans-serif;font-size:14px;
  overflow:hidden;padding:10px 5px;word-break:normal;}
.tg th{border-color:black;border-style:solid;border-width:1px;font-family:Arial, sans-serif;font-size:14px;
  font-weight:normal;overflow:hidden;padding:10px 5px;word-break:normal;}
.tg .tg-e5wy{background-color:#8584b6;color:#000000;font-weight:bold;text-align:left;vertical-align:top}
.tg .tg-u4ra{background-color:#ffffff;border-color:#efefef;color:#000000;text-align:left;vertical-align:top}
</style>
<table class="tg">
<thead>
  <tr>
    <th class="tg-e5wy">Text File</th>
    <th class="tg-e5wy">Field #1</th>
    <th class="tg-e5wy">Field #2</th>
    <th class="tg-e5wy">Field #3</th>
    <th class="tg-e5wy">Field #4</th>
    <th class="tg-e5wy">Field #5</th>
  </tr>
</thead>
<tbody>
  <tr>
    <td class="tg-u4ra">Barcode Identification</td>
    <td class="tg-u4ra">subject #</td>
    <td class="tg-u4ra">barcode #</td>
    <td class="tg-u4ra"></td>
    <td class="tg-u4ra"></td>
    <td class="tg-u4ra"></td>
  </tr>
  <tr>
    <td class="tg-u4ra">Landmark Groundtruth</td>
    <td class="tg-u4ra">subject #</td>
    <td class="tg-u4ra">x [m]</td>
    <td class="tg-u4ra">y [m]</td>
    <td class="tg-u4ra">x std-dev [m]</td>
    <td class="tg-u4ra">y std-dev [m]</td>
  </tr>
  <tr>
    <td class="tg-u4ra">Robot Groundtruth (one for each robot)</td>
    <td class="tg-u4ra">time [s]</td>
    <td class="tg-u4ra">x [m]</td>
    <td class="tg-u4ra">y [m]</td>
    <td class="tg-u4ra">θ [rad]</td>
    <td class="tg-u4ra"></td>
  </tr>
  <tr>
    <td class="tg-u4ra">Robot Odometry (one for each robot)</td>
    <td class="tg-u4ra">time [s]</td>
    <td class="tg-u4ra">v [m/s]</td>
    <td class="tg-u4ra">ω [rad/s]</td>
    <td class="tg-u4ra"></td>
    <td class="tg-u4ra"></td>
  </tr>
  <tr>
    <td class="tg-u4ra">Measurement Files (one for each robot)</td>
    <td class="tg-u4ra">time [s]</td>
    <td class="tg-u4ra">subject #</td>
    <td class="tg-u4ra">r [m]</td>
    <td class="tg-u4ra">φ [rad]</td>
    <td class="tg-u4ra"></td>
  </tr>
</tbody>
</table>