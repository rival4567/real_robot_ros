import React from 'react';
import * as ROS3D from 'ros3d';
import * as ROSLIB from 'roslib';
import AutoRos from './AutoRos';


/**
 * Setup all visualization elements when the page is loaded. 
 */

export default class ROSLoader2 extends React.Component {

    constructor(props) {
        super(props);

        this.viewerRef = React.createRef();
        this.viewer = null;
        this.resizeObserver = null;

        this.fixed_frame = null;
        this.current_group = null;

        this.state = {
            link_group: {},
        };
        this.end_effector_link = null;
        this.start_initial_flag = true;
        this.goal_initial_flag = true;
        this.joint_states = null;
        this.start_joint_states = null;
        this.goal_joint_states = null;
        this.start_im_client = null;
        this.goal_im_client = null;
        this.tfClient = null;
        this.message_stock = new Array();

        // Connect to ROS.
        this.autoROS = new AutoRos()
        const url = 'ws://' + '10.229.199.57' + ':9090';
        this.autoROS.connect(url)
        this.ros = this.autoROS.ros

        this.joint_names = new ROSLIB.Param({
            ros: this.ros,
            name: '/joint'
        });

        this.start_pub = new ROSLIB.Topic({
            ros: this.ros,
            name: '/update_start_joint_position',
            messageType: 'std_msgs/Float64MultiArray'
        });

        this.goal_pub = new ROSLIB.Topic({
            ros: this.ros,
            name: '/update_goal_joint_position',
            messageType: 'std_msgs/Float64MultiArray'
        });

        this.im_size_pub = new ROSLIB.Topic({
            ros: this.ros,
            name: '/im_size/update',
            messageType: 'std_msgs/Float32'
        });

        this.moveit_pub = new ROSLIB.Topic({
            ros: this.ros,
            name: '/moveit_joint',
            messageType: 'rwt_moveit/MoveGroupPlan'
        });

        this.execute_pub = new ROSLIB.Topic({
            ros: this.ros,
            name: '/execute_trajectory',
            messageType: 'std_msgs/Empty'
        });


        this.joint_pub = new ROSLIB.Topic({
            ros: this.ros,
            name: '/update_joint_position',
            messageType: 'std_msgs/Float64MultiArray'
        });

        this.computefkClient = new ROSLIB.Service({
            ros : this.ros,
            name : '/compute_fk',
            serviceType : 'moveit_msgs/GetPositionFK'
        });

        this.start_initial_interactive_pub = new ROSLIB.Topic({
            ros: this.ros,
            name: '/start/initial_marker',
            messageType: 'std_msgs/String'
        });

        this.goal_initial_interactive_pub = new ROSLIB.Topic({
            ros: this.ros,
            name: '/goal/initial_marker',
            messageType: 'std_msgs/String'
        });

        this.start_interactive_pub = new ROSLIB.Topic({
            ros: this.ros,
            name: '/start/marker/feedback',
            messageType: 'visualization_msgs/InteractiveMarkerFeedback'
        });

        this.goal_interactive_pub = new ROSLIB.Topic({
            ros: this.ros,
            name: '/goal/marker/feedback',
            messageType: 'visualization_msgs/InteractiveMarkerFeedback'
        });

        this.plan_listener = new ROSLIB.Topic({
            ros: this.ros,
            name: '/stock_joint_position',
            messageType: 'std_msgs/Float64MultiArray'
        });

        this.fixed_frame_param = new ROSLIB.Param({
            ros: this.ros,
            name: '/fixed_frame'
        });
    
        this.link_group_param = new ROSLIB.Param({
            ros: this.ros,
            name: '/link_group/'
        });
    
        this.end_effector_link_param = new ROSLIB.Param({
            ros: this.ros,
            name: '/end_effector_link/'
        });
    
        // Setup listener
        this.joint_listener = new ROSLIB.Topic({
            ros : this.ros,
            name : '/joint_states',
            messageType : 'sensor_msgs/JointState'
        });
    
        this.goal_listener = new ROSLIB.Topic({
            ros : this.ros,
            name : '/goal_joint_states',
            messageType : 'sensor_msgs/JointState'
        });
    
        this.start_listener = new ROSLIB.Topic({
            ros : this.ros,
            name : '/start_joint_states',
            messageType : 'sensor_msgs/JointState'
        });    
    }

    componentDidMount() {

        this.initViewer();
        this.initResizeObserver();
        this.end_effector_link_param.get( (value) => {
            this.end_effector_link = value;
        });
    
        this.plan_listener.subscribe( (message) => {
            this.message_stock.push(message);
        });
        this.fixed_frame_param.get(this.handleFixedFrameUpdate);
        this.link_group_param.get(this.handleLinkGroupUpdate);
    }

    componentDidUpdate(prevProps) {
        const { width, height } = this.props;
        if (prevProps.width !== width || prevProps.height !== height) {
            this.viewer.resize(width, height);
        }
    }

    componentWillUnmount() {
        if (this.viewer) {
            this.viewer.stop();
        }
        if (this.resizeObserver) {
            this.resizeObserver.disconnect();
        }
    }

    initViewer() {
        const { width, height } = this.props;
        this.viewer = new ROS3D.Viewer({
            divID : this.viewerRef.current.id,
            width : width,
            height : height,
            antialias : true,
            intensity : 0.09,
            background : '#415A77',
            alpha: 0.75
        });

        this.viewer.camera.fov = 70;
        this.viewer.camera.position.set(0.9057, 1.689, 0.5714)
        this.viewer.scene.position.set(0, 0, -1)
    
        // Directional light in viewer
        this.viewer.directionalLight.color = {r: 1.0, g: 1.0, b: 1.0}
        this.viewer.directionalLight.intensity = 1.0
        this.viewer.directionalLight.position.x = 0.5
        this.viewer.directionalLight.position.y = 0.5
        this.viewer.directionalLight.position.z = 1.0
    
        // Renderer Settings
        this.viewer.renderer.shadowMap.enabled = true;
    
        this.viewer.selectableObjects.castShadow = true
    
        // const drawer = new ROS3D.MeshResource({
        //     resource: 'ablagebox.dae',
        //     path : '/meshes/',
        //     warnings : true,
        //     material : new ROS3D.makeColorMaterial(0.21176470588, 0.27058823529, 0.30980392156, 1.0)
        // });
    
        // const pedestal = new ROS3D.MeshResource({
        // resource: 'Roboter_sockel.dae',
        // path : '/meshes/',
        // warnings : true,
        // material : new ROS3D.makeColorMaterial(0.21176470588, 0.27058823529, 0.30980392156, 0.5)
        // });
    
        // drawer.position.set(0, 0, -0.41)
        // drawer.scale.set(0.3, 0.3, 0.3)
        // drawer.castShadow = true
        // drawer.receiveShadow = true
    
        // pedestal.position.set(-0.2, 0.225, -0.41)
        // pedestal.scale.set(0.0025, 0.0025, 0.0025)
        // pedestal.castShadow = true
        // pedestal.receiveShadow = true
    
        // this.viewer.addObject(drawer);
        // this.viewer.addObject(pedestal);
    
        // Add a grid.
        this.viewer.addObject(new ROS3D.Grid());
    }

    initResizeObserver() {
        const targetNode = this.viewerRef.current;
        this.resizeObserver = new ResizeObserver((entries) => {
            const { width, height } = entries[0].contentRect;
            this.viewer.resize(width, height);
        });
        this.resizeObserver.observe(targetNode);
    }

    handleFixedFrameUpdate = (value) => {
        //Setup a client to listen to TFs.
        this.fixed_frame = value;
        this.tfClient = new ROSLIB.TFClient({
            ros : this.ros,
            fixedFrame : this.fixed_frame,
            angularThres : 0.01,
            transThres : 0.01,
            rate : 60.0
        });
        // Setup the marker client.
        this.start_im_client = new ROS3D.InteractiveMarkerClient({
            ros : this.ros,
            tfClient : this.tfClient,
            hidden : true,
            topic : '/start/marker',
            camera : this.viewer.camera,
            rootObject : this.viewer.selectableObjects
        });
        this.goal_im_client = new ROS3D.InteractiveMarkerClient({
            ros : this.ros,
            tfClient : this.tfClient,
            hidden : true,
            topic : '/goal/marker',
            camera : this.viewer.camera,
            rootObject : this.viewer.selectableObjects
        });
    }

    handleLinkGroupUpdate = (value) => {
        this.setState({ link_group : value });
        setTimeout( () => {
            this.createSliderView();

            this.joint_listener.subscribe( (message) => {
                this.joint_states = message;
            });

            this.start_listener.subscribe( (message) => {
                const link_group = this.state.link_group;
                this.start_joint_states = message;
                if(document.querySelector('input[name="manip"]').checked) return;

                let fk_link_name;

                if (this.end_effector_link[this.current_group] === null) {
                    fk_link_name = "schunk_gripper";
                }
                else {
                    fk_link_name = this.end_effector_link[this.current_group];
                }

                // Update interactive marker poisition
                let request = new ROSLIB.ServiceRequest({
                    header: {
                        seq: 0,
                        stamp: 0,
                        frame_id: this.fixed_frame
                    },
                    fk_link_names: [fk_link_name],
                    robot_state: {
                        joint_state: this.start_joint_states
                    }
                });

                this.computefkClient.callService(request, (result) => {
                    
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
                    this.start_interactive_pub.publish(interactive_msg);
                });


                for (let idx = 0; idx < this.start_joint_states.name.length; idx++) {                
                    for (let jdx = 0; jdx < link_group[this.current_group].length; jdx++) {
                        if (link_group[this.current_group][jdx] === this.start_joint_states.name[idx]) {
                            let min = document.querySelector('input#' + link_group[this.current_group][jdx]).getAttribute("min");
                            let max = document.querySelector('input#' + link_group[this.current_group][jdx]).getAttribute("max");
                            let percent = parseInt((this.start_joint_states.position[idx] - min)/(max - min) * 100);
                            const html = "<div role='slider' style='width: " + percent + "%;' aria-valuenow=" + this.start_joint_states.position[idx] + " aria-value-text='0.5' title=" + this.start_joint_states.position[idx] + "> " + this.start_joint_states.position[idx] + " </div>";
                            document.querySelector('input#' + link_group[this.current_group][jdx]).setAttribute("value", this.start_joint_states.position[idx]);
                            document.querySelector('input#' + link_group[this.current_group][jdx]).parentElement.innerHTML = html;
                            document.querySelector('input#' + link_group[this.current_group][jdx]).setAttribute("value", this.start_joint_states.position[idx]);
                            break;
                        }
                    }
                }
                                                

            });

            this.goal_listener.subscribe( (message) => {
                const link_group = this.state.link_group;
                this.goal_joint_states = message;
                if(document.querySelector('input[name="manip"]').checked === false) return;

                let fk_link_name;

                if (this.end_effector_link[this.current_group] == null) {
                    fk_link_name = "schunk_gripper";
                }
                else {
                    fk_link_name = this.end_effector_link[this.current_group];
                }

                // Update interactive marker poisition
                let request = new ROSLIB.ServiceRequest({
                    header: {
                        seq: 0,
                        stamp: 0,
                        frame_id: this.fixed_frame
                    },
                    fk_link_names: [fk_link_name],
                    robot_state: {
                        joint_state: this.goal_joint_states
                    }
                });

                this.computefkClient.callService(request, (result) => {
                        
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
                    this.goal_interactive_pub.publish(interactive_msg);
                    this.goal_initial_flag = false;
                });

                for (let idx = 0; idx < this.goal_joint_states.name.length; idx++) {                
                    for (let jdx = 0; jdx < link_group[this.current_group].length; jdx++) {
                        if (link_group[this.current_group][jdx] === this.goal_joint_states.name[idx]) {
                            const min = document.querySelector('input#' + link_group[this.current_group][jdx]).getAttribute("min");
                            const max = document.querySelector('input#' + link_group[this.current_group][jdx]).getAttribute("max");
                            let percent = parseInt((this.goal_joint_states.position[idx] - min)/(max - min) * 100);
                            const html = "<div style='width: " + percent + "%;' aria-valuenow=" + this.goal_joint_states.position[idx] + " aria-value-text='0.5' title=" + this.goal_joint_states.position[idx] + "></div>";
                            document.querySelector('input#' + link_group[this.current_group][jdx]).setAttribute("value", this.goal_joint_states.position[idx]);
                            document.querySelector('input#' + link_group[this.current_group][jdx]).parentElement.innerHTML = html;
                            document.querySelector('input#' + link_group[this.current_group][jdx]).setAttribute("value", this.goal_joint_states.position[idx]);
                            break;
                        }
                    }
                }                          
            });
            this.create_joint_position_msg(1, true);
        }, 3000);

        // setTimeout( () => {
        //     // Setup the URDF client.
        //     let goalState = new ROS3D.UrdfClient({
        //         ros : this.ros,
        //         tfPrefix : 'goal',
        //         tfClient : this.tfClient,
        //         param : 'robot_description',
        //         path : 'robot_description',
        //         rootObject : this.viewer.scene,
        //         colorMaterial: new ROS3D.makeColorMaterial(1.0, 0, 0, 0.75)
        //     });

        //     let urdfClient = new ROS3D.UrdfClient({
        //         ros : this.ros,
        //         tfClient : this.tfClient,
        //         param : 'robot_description',
        //         path : 'robot_description',
        //         rootObject : this.viewer.scene,
        //     });

        //     let startState = new ROS3D.UrdfClient({
        //         ros : this.ros,
        //         tfPrefix : 'start',
        //         color : 0x00df00,
        //         tfClient : this.tfClient,
        //         param : 'robot_description',
        //         path : 'robot_description',
        //         rootObject : this.viewer.scene,
        //         colorMaterial: new ROS3D.makeColorMaterial(0, 1.0, 0, 0.75) 
        //     });


        //     document.querySelector('#start_state').addEventListener("change", function() {
        //         if(this.checked) {
        //             if (document.querySelectorAll('input[name="manip"]')[0].checked) {
        //                 this.start_im_client.rootObject.children[0].visible = true;
        //             }
        //             this.viewer.scene.add(startState.urdf);
        //         }
        //         else {
        //             this.start_im_client.rootObject.children[0].visible = false;
        //             this.viewer.scene.remove(startState.urdf);
        //         }
        //     });

        //     document.querySelector('#goal_state').addEventListener("change", function() {
        //         if(this.checked) {
        //             if (document.querySelectorAll('input[name="manip"]')[1].checked) {
        //                 this.goal_im_client.rootObject.children[1].visible = true;
        //             }
        //             this.viewer.scene.add(goalState.urdf);
        //         }
        //         else {
        //             this.goal_im_client.rootObject.children[1].visible = false;
        //             this.viewer.scene.remove(goalState.urdf);
        //         }
        //     });

        //     document.querySelectorAll('input[name="manip"]').forEach(input => {
        //         input.addEventListener("change", function() {
        //         if(this.value == "0"){
        //             if(document.querySelector('#start_state').checked) {
        //                 this.start_im_client.rootObject.children[0].visible = true;
        //             }
        //             this.goal_im_client.rootObject.children[1].visible = false;
        //         } else {
        //             if(document.querySelector('#goal_state').checked) {
        //                 this.goal_im_client.rootObject.children[1].visible = true;
        //             }
        //             this.start_im_client.rootObject.children[0].visible = false;
        //         }
        //     });
        //     });

        // }, 1500);
    }


    handleGroupChange = () => {
        let selector = document.querySelectorAll("select#group option");
        selector.forEach(selection => {
            document.querySelector("#" + selection.value).style.display="none";
        });
        let group = document.querySelector("select#group").value;
        this.current_group = group;
        document.querySelector("#" + group).style.display="block";
        let msg = new ROSLIB.Message({
            data: this.current_group
        });
        // inform current group
        this.start_initial_interactive_pub.publish(msg);
        this.goal_initial_interactive_pub.publish(msg);
        this.start_initial_flag = true;
        this.goal_initial_flag = true;
        this.create_joint_position_msg(1, true);
    }

    create_joint_position_msg = (type, plan_only) => {

        let positions = new Array();
        let start_positions = new Array();
        let goal_positions = new Array();
        let dims = new Array();

        document.querySelectorAll("#" + this.current_group + " > label").forEach(label => {
            let dim = new ROSLIB.Message({
                label: label.getAttribute("id").split("-")[0],
                size: label.getAttribute("id").split("-")[0].length,
                stride: label.getAttribute("id").split("-")[0].length
            });
            dims.push(dim);
            if (type === 0) {
                for (let idx = 0; idx < this.start_joint_states.name.length;idx++) {
                    if (this.start_joint_states.name[idx] === dim.label) {
                        start_positions.push(this.start_joint_states.position[idx]);
                        goal_positions.push(this.goal_joint_states.position[idx]);
                        break;
                    }
                }
            }
            else {
            positions.push(parseFloat(label.nextElementSibling.nextElementSibling.getAttribute("aria-valuenow")));
            }
        });

        let msg;
        if (type === 0) {
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
                group_name: this.current_group
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

    callback = () => {
        let msg = this.create_joint_position_msg(1, true);
        if(document.querySelector('input[name="manip"]').checked === false) {
            this.start_pub.publish(msg);
        }
        else {
            this.goal_pub.publish(msg);
        }
    }

    // create joint_publisher
    createSliderView = () => {
        const link_group = this.state.link_group;
        let idx = 0;
        for (let group_name in link_group) {
            let group_div = document.createElement('div');
            group_div.id = group_name
            document.querySelector("#slider-pane").appendChild(group_div);
            if (idx !== 0) {
                document.querySelector("#" + group_name).style.display="none";
            }
            else {
                this.current_group = group_name;
                idx++;
            }
        }
        this.joint_names.get( (value) => {

            let names = value.names;
            for (let group_name in link_group) {
                for (let idx = 0;idx < names.length;idx++) {
                    if (link_group[group_name].indexOf(names[idx]) !== -1) {
                        let child = document.createElement('label');
                        child.id = names[idx];
                        child.for = names[idx];
                        child.textContent = names[idx];
                        let child2 = document.createElement('input');
                        child2.type = "range";
                        child2.name = names[idx];
                        child2.id = names[idx];
                        child2.value = 0;
                        child2.max = value.names[idx];
                        console.log(typeof(child2.max))
                        child2.min = eval("value." + names[idx] + ".min");
                        child2.step = 0.000001;
                        child2.setAttribute("data-highlight", "true");
                        child2.setAttribute("data-mini", "true");
                        child2.addEventListener("change", this.callback);
                        document.querySelector("#" + group_name).appendChild(child);
                        document.querySelector("#" + group_name).appendChild(child2);
                        
                    }
                }
            }
            let msg = new ROSLIB.Message({
                data: this.current_group
            });
            this.start_initial_interactive_pub.publish(msg);
            this.goal_initial_interactive_pub.publish(msg);
        });
    }
    


    render() {


        const link_group = this.state.link_group;
        const joint = this.state.joint_name;
        const group_name = Object.keys(link_group).map( (key) => {
            return <option key={key} value={key}>{key}</option>
        });



        return (
            <>
                <div className="viewer">
                    <div id="rosViewer" style={{ width: '100%', height: '100%' }} ref={this.viewerRef}></div>
                </div>
                <div className='control-panel'>
                    {Object.keys(link_group).length === 0 ? (
                        <div className='loading'>Loading...</div>
                    ) : (<>
                    <select id="group" name="group>" onChange={this.handleGroupChange}>
                        {group_name}
                    </select>
                    <div id="slider-pane">
                        <table>
                            <thead>
                                <tr><td></td><td>startState</td><td>goalState</td></tr>
                            </thead>
                            <tbody>
                                <tr>
                                    <td>
                                    View
                                    </td>
                                    <td>
                                    <input type="checkbox" name="start_state" id="start_state"/>
                                    </td>
                                    <td>
                                    <input type="checkbox" name="goal_state" id="goal_state"/>
                                    </td>
                                </tr>  
                                <tr>
                                    <td>
                                    Maniplation
                                    </td>
                                    <td>
                                    <input type="radio" name="manip" id="manip" value="0" defaultChecked/>
                                    </td>
                                    <td>
                                    <input type="radio" name="manip" id="manip" value="1"/>
                                    </td>
                                </tr>
                            </tbody>

                        </table>
                    </div>
                    </>
                    )}
                </div>
            </>
            );

        }
}
