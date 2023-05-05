import React, { Component } from 'react';
import * as ROS3D from 'ros3d';
import * as ROSLIB from 'roslib';
import AutoRos from './AutoRos';


/**
 * Setup all visualization elements when the page is loaded. 
 */

export default class ROSLoader extends Component {    
    
    render() {

    let fixed_frame;
    let current_group;
    let link_group;
    let end_effector_link;
    let start_initial_flag = true;
    let goal_initial_flag = true;
    let joint_states;
    let start_joint_states;
    let goal_joint_states;
    let start_im_client;
    let goal_im_client;
    let tfClient;
    let message_stock = new Array();

    // Connect to ROS.
    const autoROS = new AutoRos()
    const url = 'ws://' + '10.229.199.57' + ':9090';
    autoROS.connect(url)
    const ros = autoROS.ros

    let joint_names = new ROSLIB.Param({
        ros: ros,
        name: '/joint'
    });

    const start_pub = new ROSLIB.Topic({
        ros: ros,
        name: '/update_start_joint_position',
        messageType: 'std_msgs/Float64MultiArray'
    });

    const goal_pub = new ROSLIB.Topic({
        ros: ros,
        name: '/update_goal_joint_position',
        messageType: 'std_msgs/Float64MultiArray'
    });

    const im_size_pub = new ROSLIB.Topic({
        ros: ros,
        name: '/im_size/update',
        messageType: 'std_msgs/Float32'
    });

    const moveit_pub = new ROSLIB.Topic({
        ros: ros,
        name: '/moveit_joint',
        messageType: 'rwt_moveit/MoveGroupPlan'
    });

    const execute_pub = new ROSLIB.Topic({
        ros: ros,
        name: '/execute_trajectory',
        messageType: 'std_msgs/Empty'
    });


    const joint_pub = new ROSLIB.Topic({
        ros: ros,
        name: '/update_joint_position',
        messageType: 'std_msgs/Float64MultiArray'
    });

    const computefkClient = new ROSLIB.Service({
        ros : ros,
        name : '/compute_fk',
        serviceType : 'moveit_msgs/GetPositionFK'
    });

    const start_initial_interactive_pub = new ROSLIB.Topic({
        ros: ros,
        name: '/start/initial_marker',
        messageType: 'std_msgs/String'
    });

    const goal_initial_interactive_pub = new ROSLIB.Topic({
        ros: ros,
        name: '/goal/initial_marker',
        messageType: 'std_msgs/String'
    });

    const start_interactive_pub = new ROSLIB.Topic({
        ros: ros,
        name: '/start/marker/feedback',
        messageType: 'visualization_msgs/InteractiveMarkerFeedback'
    });

    const goal_interactive_pub = new ROSLIB.Topic({
        ros: ros,
        name: '/goal/marker/feedback',
        messageType: 'visualization_msgs/InteractiveMarkerFeedback'
    });

    const plan_listener = new ROSLIB.Topic({
        ros: ros,
        name: '/stock_joint_position',
        messageType: 'std_msgs/Float64MultiArray'
    });

    // Create the main viewer.
    const body = document.body;
    const html = document.documentElement;
    const width = parseInt(document.querySelector("#main-content").offsetWidth);
    const height = Math.max( body.scrollHeight, body.offsetHeight, 
        html.clientHeight, html.scrollHeight, html.offsetHeight );

    const viewer = new ROS3D.Viewer({
        divID : 'urdf',
        width : width * 0.8,
        height : height * 0.8,
        antialias : true,
        intensity : 0.09,
        background : '#415A77',
        alpha: 0.75
    });

    viewer.camera.fov = 70;
    viewer.camera.position.set(0.9057, 1.689, 0.5714)
    viewer.scene.position.set(0, 0, -1)

    // Directional light in viewer
    viewer.directionalLight.color = {r: 1.0, g: 1.0, b: 1.0}
    viewer.directionalLight.intensity = 1.0
    viewer.directionalLight.position.x = 0.5
    viewer.directionalLight.position.y = 0.5
    viewer.directionalLight.position.z = 1.0

    // Renderer Settings
    viewer.renderer.shadowMap.enabled = true;
    console.log(viewer.renderer)
    console.log(window)

    viewer.selectableObjects.castShadow = true

    const drawer = new ROS3D.MeshResource({
        resource: 'ablagebox.dae',
        path : '/meshes/',
        warnings : true,
        material : new ROS3D.makeColorMaterial(0.21176470588, 0.27058823529, 0.30980392156, 1.0)
    });

    const pedestal = new ROS3D.MeshResource({
    resource: 'Roboter_sockel.dae',
    path : '/meshes/',
    warnings : true,
    material : new ROS3D.makeColorMaterial(0.21176470588, 0.27058823529, 0.30980392156, 0.5)

    });

    drawer.position.set(0, 0, -0.41)
    drawer.scale.set(0.3, 0.3, 0.3)
    drawer.castShadow = true
    drawer.receiveShadow = true

    pedestal.position.set(-0.2, 0.225, -0.41)
    pedestal.scale.set(0.0025, 0.0025, 0.0025)
    pedestal.castShadow = true
    pedestal.receiveShadow = true

    viewer.addObject(drawer);
    viewer.addObject(pedestal);

    // Add a grid.
    viewer.addObject(new ROS3D.Grid());


    let fixed_frame_param = new ROSLIB.Param({
        ros: ros,
        name: '/fixed_frame'
    });

    let link_group_param = new ROSLIB.Param({
        ros: ros,
        name: '/link_group/'
    });

    let end_effector_link_param = new ROSLIB.Param({
        ros: ros,
        name: '/end_effector_link/'
    });

    // Setup listener
    let joint_listener = new ROSLIB.Topic({
        ros : ros,
        name : '/joint_states',
        messageType : 'sensor_msgs/JointState'
    });

    let goal_listener = new ROSLIB.Topic({
        ros : ros,
        name : '/goal_joint_states',
        messageType : 'sensor_msgs/JointState'
    });

    let start_listener = new ROSLIB.Topic({
        ros : ros,
        name : '/start_joint_states',
        messageType : 'sensor_msgs/JointState'
    });


    // Setup a client to listen to TFs.
    fixed_frame_param.get(function(value) {
        fixed_frame = value;
        tfClient = new ROSLIB.TFClient({
            ros : ros,
            fixedFrame : fixed_frame,
            angularThres : 0.01,
            transThres : 0.01,
            rate : 60.0
        });
        // Setup the marker client.
        const start_im_client = new ROS3D.InteractiveMarkerClient({
            ros : ros,
            tfClient : tfClient,
            hidden : true,
            topic : '/start/marker',
            camera : viewer.camera,
            rootObject : viewer.selectableObjects
        });
        const goal_im_client = new ROS3D.InteractiveMarkerClient({
            ros : ros,
            tfClient : tfClient,
            hidden : true,
            topic : '/goal/marker',
            camera : viewer.camera,
            rootObject : viewer.selectableObjects
        });
    });

    link_group_param.get(function(value) {
        link_group = value;
        for (let group_name in link_group) {
            let group_element = document.createElement('option');
            group_element.value = group_name;
            group_element.textContent = group_name;
            document.querySelector('#group').appendChild(group_element);
        }
        document.querySelector("select#group").addEventListener("change", function() {
            let selector = document.querySelectorAll("select#group option");
            selector.forEach(selection => {
                document.querySelector("#" + selection.value).style.display="none";
            });
            let group = document.querySelector("select#group").value;
            current_group = group;
            document.querySelector("#" + group).style.display="block";
            let msg = new ROSLIB.Message({
                data: current_group
            });
            // inform current group
            start_initial_interactive_pub.publish(msg);
            goal_initial_interactive_pub.publish(msg);
            start_initial_flag = true;
            goal_initial_flag = true;
            create_joint_position_msg(1, true);
        });

        setTimeout(function() {
            createSliderView();

            joint_listener.subscribe(function(message) {
                joint_states = message;
            });


            start_listener.subscribe(function(message) {
                start_joint_states = message;
                if(document.querySelector('input[name="manip"]').checked) return;

                let fk_link_name;

                if (end_effector_link[current_group] == null) {
                    fk_link_name = "schunk_gripper";
                }
                else {
                    fk_link_name = end_effector_link[current_group];
                }

                // Update interactive marker poisition
                let request = new ROSLIB.ServiceRequest({
                    header: {
                        seq: 0,
                        stamp: 0,
                        frame_id: fixed_frame
                    },
                    fk_link_names: [fk_link_name],
                    robot_state: {
                        joint_state: start_joint_states
                    }
                });

                computefkClient.callService(request, function(result) {
                    
                    let interactive_msg = new ROSLIB.Message({
                        marker_name: "start",
                        event_type: 0,
                        pose: {
                            position: {
                                x: result.pose_stamped[0].pose.position.x,
                                y: result.pose_stamped[0].pose.position.y,
                                z: result.pose_stamped[0].pose.position.z
                            },
                            orientation: {
                                x: result.pose_stamped[0].pose.orientation.x,
                                y: result.pose_stamped[0].pose.orientation.y,
                                z: result.pose_stamped[0].pose.orientation.z,
                                w: result.pose_stamped[0].pose.orientation.w
                            }
                        }
                    });
                    start_interactive_pub.publish(interactive_msg);
                });


                for (let idx = 0; idx < start_joint_states.name.length; idx++) {
                    let joint_name, joint_num;
                
                    for (let jdx = 0; jdx < link_group[current_group].length; jdx++) {
                        if (link_group[current_group][jdx] == start_joint_states.name[idx]) {
                            let min = document.querySelector('input#' + link_group[current_group][jdx]).getAttribute("min");
                            let max = document.querySelector('input#' + link_group[current_group][jdx]).getAttribute("max");
                            let percent = parseInt((start_joint_states.position[idx] - min)/(max - min) * 100);
                            const html = "<div style='width: " + percent + "%;' aria-valuenow=" + start_joint_states.position[idx] + " aria-value-text='0.5' title=" + start_joint_states.position[idx] + "></div>";
                            document.querySelector('input#' + link_group[current_group][jdx]).setAttribute("value", start_joint_states.position[idx]);
                            document.querySelector('input#' + link_group[current_group][jdx]).innerHTML = html;
                            document.querySelector('input#' + link_group[current_group][jdx]).setAttribute("value", start_joint_states.position[idx]);
                            break;
                        }
                    }
                }
                                                

            });

            goal_listener.subscribe(function(message) {
                goal_joint_states = message;
                if(document.querySelector('input[name="manip"]').checked == false) return;

                let fk_link_name;

                if (end_effector_link[current_group] == null) {
                    fk_link_name = "schunk_gripper";
                }
                else {
                    fk_link_name = end_effector_link[current_group];
                }

                // Update interactive marker poisition
                let request = new ROSLIB.ServiceRequest({
                    header: {
                        seq: 0,
                        stamp: 0,
                        frame_id: fixed_frame
                    },
                    fk_link_names: [fk_link_name],
                    robot_state: {
                        joint_state: goal_joint_states
                    }
                });

                computefkClient.callService(request, function(result) {
                        
                    let interactive_msg = new ROSLIB.Message({
                        marker_name: "goal",
                        event_type: 0,
                        pose: {
                            position: {
                                x: result.pose_stamped[0].pose.position.x,
                                y: result.pose_stamped[0].pose.position.y,
                                z: result.pose_stamped[0].pose.position.z
                            },
                            orientation: {
                                x: result.pose_stamped[0].pose.orientation.x,
                                y: result.pose_stamped[0].pose.orientation.y,
                                z: result.pose_stamped[0].pose.orientation.z,
                                w: result.pose_stamped[0].pose.orientation.w
                            }
                        }
                    });
                    goal_interactive_pub.publish(interactive_msg);
                    goal_initial_flag = false;
                });

                for (let idx = 0; idx < goal_joint_states.name.length; idx++) {
                    let joint_name, joint_num;
                
                    for (let jdx = 0; jdx < link_group[current_group].length; jdx++) {
                        if (link_group[current_group][jdx] == goal_joint_states.name[idx]) {
                            let min = document.querySelector('input#' + link_group[current_group][jdx]).getAttribute("min");
                            let max = document.querySelector('input#' + link_group[current_group][jdx]).getAttribute("max");
                            let percent = parseInt((goal_joint_states.position[idx] - min)/(max - min) * 100);
                            const html = "<div style='width: " + percent + "%;' aria-valuenow=" + goal_joint_states.position[idx] + " aria-value-text='0.5' title=" + goal_joint_states.position[idx] + "></div>";
                            document.querySelector('input#' + link_group[current_group][jdx]).setAttribute("value", goal_joint_states.position[idx]);
                            document.querySelector('input#' + link_group[current_group][jdx]).innerHTML = html;
                            document.querySelector('input#' + link_group[current_group][jdx]).setAttribute("value", goal_joint_states.position[idx]);
                            break;
                        }
                    }
                }                          
            });

            create_joint_position_msg(1, true);
        }, 3000);

        setTimeout(function() {
            // Setup the URDF client.
            let goalState = new ROS3D.UrdfClient({
                ros : ros,
                tfPrefix : 'goal',
                tfClient : tfClient,
                param : 'robot_description',
                path : 'robot_description',
                rootObject : viewer.scene,
                colorMaterial: new ROS3D.makeColorMaterial(1.0, 0, 0, 0.75)
            });

            let urdfClient = new ROS3D.UrdfClient({
                ros : ros,
                tfClient : tfClient,
                param : 'robot_description',
                path : 'robot_description',
                rootObject : viewer.scene,
            });

            let startState = new ROS3D.UrdfClient({
                ros : ros,
                tfPrefix : 'start',
                color : 0x00df00,
                tfClient : tfClient,
                param : 'robot_description',
                path : 'robot_description',
                rootObject : viewer.scene,
                colorMaterial: new ROS3D.makeColorMaterial(0, 1.0, 0, 0.75) 
            });


            document.querySelector('#start_state').addEventListener("change", function() {
                if(this.checked) {
                    if (document.querySelectorAll('input[name="manip"]')[0].checked) {
                        start_im_client.rootObject.children[0].visible = true;
                    }
                    viewer.scene.add(startState.urdf);
                }
                else {
                    start_im_client.rootObject.children[0].visible = false;
                    viewer.scene.remove(startState.urdf);
                }
            });

            document.querySelector('#goal_state').addEventListener("change", function() {
                if(this.checked) {
                    if (document.querySelectorAll('input[name="manip"]')[1].checked) {
                        goal_im_client.rootObject.children[1].visible = true;
                    }
                    viewer.scene.add(goalState.urdf);
                }
                else {
                    goal_im_client.rootObject.children[1].visible = false;
                    viewer.scene.remove(goalState.urdf);
                }
            });

            document.querySelectorAll('input[name="manip"]').forEach(input => {
                input.addEventListener("change", function() {
                if(this.value == "0"){
                    if(document.querySelector('#start_state').checked) {
                        start_im_client.rootObject.children[0].visible = true;
                    }
                    goal_im_client.rootObject.children[1].visible = false;
                } else {
                    if(document.querySelector('#goal_state').checked) {
                        goal_im_client.rootObject.children[1].visible = true;
                    }
                    start_im_client.rootObject.children[0].visible = false;
                }
            });
            });

        }, 1500);
    });

    end_effector_link_param.get(function(value) {
        end_effector_link = value;
    });

    plan_listener.subscribe(function(message) {
        message_stock.push(message);
    });

    document.querySelector("button#init").addEventListener("pointerdown", function() {

        let positions = new Array();
        let dims = new Array();
        document.querySelectorAll("#" + current_group + " > label").forEach(label => {
            let dim = new ROSLIB.Message({
                label: (label.getAttribute("id").split("-")[0]),
                size: (label.getAttribute("id").split("-")[0]).length,
                stride: (label.getAttribute("id").split("-")[0]).length
            });
            dims.push(dim);
            for (let i = 0; i < joint_states.name.length;i++) {
                if (joint_states.name[i] == dim.label) {
                    positions.push(joint_states.position[i]);
                    break;
                }
            }
        });

        let msg;
        msg = new ROSLIB.Message({
            layout: {
                dim: dims,
                data_offset: 0
            },
            data: positions
        });
        if(document.querySelector('input[name="manip"]').checked == false) {
            start_pub.publish(msg);
        }
        else {
            goal_pub.publish(msg);
        }
    });

    document.querySelector("button#preview").addEventListener("pointerdown", function() {
        if(message_stock != null) {
            let idx = 0;
            let tmp_start_joint_states = start_joint_states;

            let timer = setInterval(function() {
                start_pub.publish(message_stock[idx]);
                idx++;
                if(idx == message_stock.length) {

                    let positions = new Array();
                    let dims = new Array();

                    document.querySelectorAll("#" + current_group + " > label").forEach(label => {
                        let dim = new ROSLIB.Message({
                            label: (label.attr("id").split("-")[0]),
                            size: (label.attr("id").split("-")[0]).length,
                            stride: (label.attr("id").split("-")[0]).length
                        });
                        dims.push(dim);
                        for (let idx = 0; idx < tmp_start_joint_states.name.length;idx++) {
                            if (tmp_start_joint_states.name[idx] == dim.label) {
                                positions.push(tmp_start_joint_states.position[idx]);
                                break;
                            }
                        }
                    });

                    let msg;
                    msg = new ROSLIB.Message({
                        layout: {
                            dim: dims,
                            data_offset: 0
                        },
                        data: positions
                    });
                    
                    start_pub.publish(msg);
                    clearInterval(timer);
                }
            },100);
        }
    });

    document.querySelector("button#moveit").addEventListener("pointerdown", function() {
        let msg = create_joint_position_msg(0,false);
        moveit_pub.publish(msg);
    });

    document.querySelector("button#plan").addEventListener("pointerdown", function() {
        let msg = create_joint_position_msg(0,true);
        moveit_pub.publish(msg);
        const display_planned_path = new ROS3D.Path({
            ros : ros,
            topic : '/kr1410/trajectory_line',
            tfClient : tfClient,
            rootObject : viewer.scene
        });
    });

    document.querySelector("button#execute").addEventListener("pointerdown", function() {
        if(message_stock != null) {
            let sim_mode = new ROSLIB.Param({
                ros: ros,
                name: '/sim_mode'
            });
            sim_mode.get(function(value) {
                if (value == true) {
                    let timer = setInterval(joint_publish, 100);
                    function joint_publish() {
                        if(message_stock.length == 0) {
                            clearInterval(timer);
                        }
                        else {
                            joint_pub.publish(message_stock.shift());
                        }
                    }
                }
                else {
                    let msg = new ROSLIB.Message({
                    });
                    execute_pub.publish(msg);
                }                
            });
        }
    });

    function create_joint_position_msg(type, plan_only) {

        let positions = new Array();
        let start_positions = new Array();
        let goal_positions = new Array();
        let dims = new Array();

        document.querySelectorAll("#" + current_group + " > label").forEach(label => {
            let dim = new ROSLIB.Message({
                label: label.getAttribute("id").split("-")[0],
                size: label.getAttribute("id").split("-")[0].length,
                stride: label.getAttribute("id").split("-")[0].length
            });
            dims.push(dim);
            if (type == 0) {
                for (let i = 0; i < start_joint_states.name.length;i++) {
                    if (start_joint_states.name[i] == dim.label) {
                        start_positions.push(start_joint_states.position[i]);
                        goal_positions.push(goal_joint_states.position[i]);
                        break;
                    }
                }
            }
            else {
            positions.push(parseFloat(label.nextElementSibling.firstElementChild.getAttribute("aria-valuenow")));
            }
        });

        let msg;
        if (type == 0) {
            msg = new ROSLIB.Message({
                start_joint: {
                    layout: {
                        dim: dims,
                        data_offset: 0
                    },
                    data: start_positions
                },
                goal_joint: {
                    layout: {
                        dim: dims,
                        data_offset: 0
                    },
                    data: goal_positions
                },
                plan_only: plan_only,
                group_name: current_group
            });
        }
        else {
            msg = new ROSLIB.Message({
                layout: {
                    dim: dims,
                    data_offset: 0
                },
                data: positions
            });
        }
        return msg;
    }

    function callback() {
        let msg = create_joint_position_msg(1, true);
        if(document.querySelector('input[name="manip"]').checked == false) {
            start_pub.publish(msg);
        }
        else {
            goal_pub.publish(msg);
        }
    }


    // create joint_publisher
    function createSliderView() {
        let i = 0;
        for (let group_name in link_group) {
            let group_div = document.createElement('div');
            group_div.id = group_name
            document.querySelector("#slider-pane").appendChild(group_div);
            if (i != 0) {
                document.querySelector("#" + group_name).style.display="none";
            }
            else {
                current_group = group_name;
                i++;
            }
        }
        joint_names.get(function(value) {
            let names = value.names;
            for (let group_name in link_group) {
                for (let idx = 0;idx < names.length;idx++) {
                    if (link_group[group_name].indexOf(names[idx]) != -1) {
                        let child = document.createElement('label');
                        child.id = names[idx];
                        child.for = names[idx];
                        child.textContent = names[idx];
                        let child2 = document.createElement('input');
                        child2.type = "range";
                        child2.name = names[idx];
                        child2.id = names[idx];
                        child2.value = 0;
                        child2.max = eval("value." + names[idx] + ".max");
                        child2.min = eval("value." + names[idx] + ".min");
                        child2.step = 0.000001;
                        child2.setAttribute("data-highlight", "true");
                        child2.setAttribute("data-mini", "true");
                        child2.addEventListener("change", callback);
                        document.querySelector("#" + group_name).appendChild(child);
                        document.querySelector("#" + group_name).appendChild(child2);
                    }
                }
            }
            let msg = new ROSLIB.Message({
                data: current_group
            });
            start_initial_interactive_pub.publish(msg);
            goal_initial_interactive_pub.publish(msg);
        });
    }

    function im_size_callback() {
        let size = parseFloat(document.querySelector("#im-size").value);
        let msg = new ROSLIB.Message({
            data: size
        });

        if (document.querySelectorAll('input[name="manip"]')[0].checked && document.querySelector('#start_state').checked) {
            im_size_pub.publish(msg);
        }
        else if(document.querySelectorAll('input[name="manip"]')[1].checked && document.querySelector('#goal_state').checked){
            im_size_pub.publish(msg);
        }
    }


    return (
        <>
            <div id="urdf" onload="init()"/>
        </>
    );
    }

}
