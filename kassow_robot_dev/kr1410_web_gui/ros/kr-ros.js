const autoROS = require('auto-ros/dist/index.js')
const ROS3D = require('ros3d');
const ROSLIB = require('roslib');

const ros = new autoROS.default()
ros.connect('ws://10.185.230.104:9090')


console.log(ROSLIB)
console.log(ROS3D)

module.exports = function rosclient() {
    // Create the main viewer.
    var viewer = new ROS3D.Viewer({
        divID : 'urdf',
        width : 800,
        height : 600,
        antialias : true,
        intensity : 0.05,
        background : '#415A77',
        alpha: 0.1
    });
    viewer.selectableObjects.castShadow = true

    // Add a grid.  
    viewer.addObject(new ROS3D.Grid({
      num_cells : 20,
      lineWidth : 0.01,
      color : '#c0c5ce',
      cellSize : 0.5
    }));

}

