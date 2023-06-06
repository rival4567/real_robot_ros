import { Component } from "react";
import ROSLIB from "roslib";
import { InteractiveMarkerClient, UrdfClient, makeColorMaterial, SceneNode, Path, Axes, Arrow } from "../ros3djs/src-esm";
import { Context } from "./ContextProvider";
import THREE from '../ros3djs/shims/three/core.js';

// TODO: Use robot_markers instead of loading 3 URDF which will save loading time
// https://github.com/RobotWebTools/ros3djs/pull/187
// UPDATE: It will still take the same time and will cost lots of time in restructuring

export default class ROSInitUrdf extends Component {
    static contextType = Context;

    constructor(props) {
        super(props);
        this.ros = props.ros;
        this.viewer = null;
        this.fixed_frame = null;
        this.fixed_frame_param = new ROSLIB.Param({
            ros: this.ros,
            name: '/fixed_frame'
        });
        this.state = {
            isAssetsLoaded : false
        }
    }

    componentDidMount() {
        this.viewer = this.context.state.viewer;
        this.fixed_frame_param.get( (value) => {
            this.handleFixedFrameUpdate(value);
            this.initUrdfClient()
            this.context.state.fixed_frame = this.fixed_frame;
        });
        setTimeout( () => {
            this.viewer.scene.add(this.urdf_scene);
            this.visualization_scene.add(this.path_scene);
            this.viewer.scene.add(this.visualization_scene);
            this.visualization_scene.scale.x = 1.001;
            this.visualization_scene.scale.y = 1.001;
            this.viewer.selectableObjects.visible = true;
            this.viewer.selectableObjects.children.map( (marker) => {
                marker.visible = false;
            });
            // TODO: Use this assets loaded to give a message in viewer using THREE.js
            this.setState({ isAssetsLoaded: true})
        }, 5000)
    }

    handleFixedFrameUpdate = (value) => {
        // Setup a client to listen to TFs.
        this.fixed_frame = value;
        this.tfClient = new ROSLIB.TFClient({
            ros : this.ros,
            fixedFrame : this.fixed_frame,
            angularThres : 0.01,
            transThres : 0.01,
            rate : 60.0
        });
        // Setup the marker client.
        this.start_im_client = new InteractiveMarkerClient({
            ros : this.ros,
            tfClient : this.tfClient,
            topic : '/start/marker',
            camera : this.viewer.camera,
            rootObject : this.viewer.selectableObjects
        });
        this.goal_im_client = new InteractiveMarkerClient({
            ros : this.ros,
            tfClient : this.tfClient,
            topic : '/goal/marker',
            camera : this.viewer.camera,
            rootObject : this.viewer.selectableObjects
        });
        this.viewer.selectableObjects.visible = false;
    }

    initUrdfClient = () => {
        this.urdf_scene = new SceneNode({
            tfClient : this.tfClient,
            frameID : this.fixed_frame,
            object : new Arrow({
                origin : new THREE.Vector3(1000, 1000, 1000),
            })
        });
        this.visualization_scene = new SceneNode({
            tfClient : this.tfClient,
            frameID : this.fixed_frame,
            object : new Arrow({
                origin : new THREE.Vector3(1000, 1000, 1000),
            })
        });
        // Setup the URDF client.
        const urdfClient = new UrdfClient({
            ros : this.ros,
            tfClient : this.tfClient,
            param : 'robot_description',
            path : 'robot_description',
            rootObject : this.urdf_scene,
        });
        this.startState = new UrdfClient({
            ros : this.ros,
            tfPrefix : 'start',
            tfClient : this.tfClient,
            param : 'robot_description',
            path : 'robot_description',
            rootObject : this.visualization_scene,
            colorMaterial: new makeColorMaterial(0.0, 1.0, 0, 0.25) 
        });
        this.goalState = new UrdfClient({
            ros : this.ros,
            tfPrefix : 'goal',
            tfClient : this.tfClient,
            param : 'robot_description',
            path : 'robot_description',
            rootObject : this.visualization_scene,
            colorMaterial: new makeColorMaterial(1.0, 0.0, 0, 0.25)
        });
        this.path_scene = new SceneNode({
            tfClient : this.tfClient,
            frameID : this.fixed_frame,
            object : new Arrow({
                origin : new THREE.Vector3(1000, 1000, 1000),
            }),
        })      
        this.display_planned_path = new Path({
            ros : this.ros,
            topic : '/kr1410/trajectory_line',
            tfClient : this.tfClient,
            rootObject : this.path_scene,
            color : 0xffffe0
        });
        this.urdf_scene.name = "Robot";
        this.visualization_scene.name = "Visualization";
    }

    render() {
        return (
            <></>
        )
    }
}