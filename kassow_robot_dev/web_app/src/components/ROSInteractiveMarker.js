import { Component, Fragment, createRef } from "react";
import ROSLIB from "roslib";
import '../styles/InteractiveMarkerStyle.css';
import { Context } from "./ContextProvider";

export default class ROSInteractiveMarker extends Component {
    static contextType = Context;

    constructor(props) {
        super(props);
        this.ros = props.ros;
        this.start_joint_states = props.start_joint_states;
        this.goal_joint_states = props.goal_joint_states;
        this.end_effector_link = null;

        this.viewer_start_ref = createRef();
        this.viewer_goal_ref = createRef();

        this.end_effector_link_param = new ROSLIB.Param({
            ros: this.ros,
            name: '/end_effector_link/'
        });
        this.computefkClient = new ROSLIB.Service({
            ros : this.ros,
            name : '/compute_fk',
            serviceType : 'moveit_msgs/GetPositionFK'
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
        this.im_size_pub = new ROSLIB.Topic({
            ros: this.ros,
            name: '/im_size/update',
            messageType: 'std_msgs/Float32'
        });
        this.state = {
            isViewStartChecked: false,
            isViewGoalChecked: false,
            toggleStartManipulation : false,
            toggleGoalManipulation : false,
        }
    }

    async componentDidMount() {
        await new Promise( (resolve, reject) => {
            this.end_effector_link_param.get( (value) => {
                this.end_effector_link = value;
            });
            let timerID = setInterval( () => {
                if(this.end_effector_link !== null){
                    clearInterval(timerID);
                    resolve();
                }
            }, 100);
            setTimeout( () => {
                clearInterval(timerID);
                // TODO: Set a state in react which will give an alert that something has gone wrong.
                reject(new Error('Failed to fetch from End Effector Link ROSParam.'));
            }, 10000);
        });
        this.context.state.selected_group.map( (group) => {
            let msg = new ROSLIB.Message({
                data: group
            });
            // inform selected groups
            this.start_initial_interactive_pub.publish(msg);
            this.goal_initial_interactive_pub.publish(msg);
            return null;
        });
        this.interactive_marker = this.context.state.viewer.selectableObjects.children;
        this.context.state.isStartManipulateActive = this.state.toggleStartManipulation;
        this.context.state.isGoalManipulateActive = this.state.toggleGoalManipulation;
    }

    componentDidUpdate (prevProps, prevState) {
        this.context.state.isStartManipulateActive = this.state.toggleStartManipulation;
        this.context.state.isGoalManipulateActive = this.state.toggleGoalManipulation;        if (prevState !== this.state ) {
            this.interactive_marker.map( (marker) => {
                if (marker.name === 'start') {
                    if (this.viewer_start_ref.current.checked) {
                        marker.visible = this.state.toggleStartManipulation;
                    } else {
                        marker.visible = false;
                        setTimeout( () => {
                            this.state.toggleStartManipulation = false;
                        }, 500);
                    }
                } else if (marker.name === 'goal') {
                    if (this.viewer_goal_ref.current.checked) {
                        marker.visible = this.state.toggleGoalManipulation;
                    } else {
                        marker.visible = false;
                        setTimeout( () => {
                            this.state.toggleGoalManipulation = false;
                        }, 500);
                    }
                }
            })
        }
    }

    handleManipulationCallback = (event) => {
        this.selected_group = this.context.state.selected_group;
        this.fixed_frame = this.context.state.fixed_frame;
        if (event.target.checked) {
            let joint_states = null;
            let interactive_pub = null;
            if(event.target.name.includes("start")) {
                joint_states = this.start_joint_states;
                interactive_pub = this.start_interactive_pub;
                this.setState((prevState) => ({
                    toggleStartManipulation: !prevState.toggleStartManipulation,
                    toggleGoalManipulation: prevState.toggleStartManipulation ? prevState.toggleGoalManipulation : false
                }));
            }
            else if(event.target.name.includes("goal")){
                joint_states = this.goal_joint_states;
                interactive_pub = this.goal_interactive_pub;
                this.setState((prevState) => ({
                    toggleGoalManipulation: !prevState.toggleGoalManipulation,
                    toggleStartManipulation: prevState.toggleGoalManipulation ? prevState.toggleStartManipulation : false
                }));
            }
            else {
                return
            }
            let fk_link_name;
            if (this.end_effector_link[this.selected_group[0]] === undefined) {
                fk_link_name = "schunk_gripper";
            }
            else {
                fk_link_name = this.end_effector_link[this.selected_group[0]];
            }
            // Update interactive marker position
            let request = new ROSLIB.ServiceRequest({
                header: {
                    seq: 0,
                    stamp: 0,
                    frame_id: this.fixed_frame
                },
                fk_link_names: [fk_link_name],
                robot_state: {
                    joint_state: joint_states
                }
            });
            this.computefkClient.callService(request, (result) => {
            
                let interactive_msg = new ROSLIB.Message({
                    marker_name: event.target.name,
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
                interactive_pub.publish(interactive_msg);

            });
            Array.from(this.props.joint_group_slider.current.children).map( (key) => {
                Array.from(key.children).map( (child) => {
                    if (child.tagName === 'INPUT') {
                        let index = joint_states.name.indexOf(child.id);
                        let isJointPresent = index !== -1;
                        if (isJointPresent) {
                            child.value = joint_states.position[index];
                        }
                    }
                })
            })
        } else {
            if(event.target.name.includes("start")) {
                this.setState({toggleStartManipulation: false})
            } else if (event.target.name.includes("goal")) {
                this.setState({toggleGoalManipulation: false})
            }
        }
    }

    handleViewCallback = (event) => {
        if(event.target.name.includes("start")) {
            this.setState({ isViewStartChecked: event.target.checked })
        } else if (event.target.name.includes("goal")) {
            this.setState({ isViewGoalChecked: event.target.checked })
        }
        if (!event.target.checked) {
            if(event.target.name.includes('start')) {
                setTimeout( () => {
                    this.setState({ toggleStartManipulation: false })
                }, 500)
            } else if (event.target.name.includes('goal')) {
                setTimeout( ()=> {
                    this.setState({ toggleGoalManipulation: false })
                }, 500)
            }
        }
        this.context.state.viewer.scene.children.map( (scene) => {
            if (scene.name === 'Visualization'){
                scene.children.map( (object) => {
                    if(object.children.length === 9) {
                        object.children.map( (link) => {
                            if (link.frameID.includes('start')) {
                                object.visible = event.target.name.includes('start') ? event.target.checked : object.visible;
                            } else if (link.frameID.includes('goal')) {
                                object.visible = event.target.name.includes('goal') ? event.target.checked : object.visible;
                            }
                        })
                    }
                })
            }
        })
    }

    handleIMCallback = (event) => {
        let msg = new ROSLIB.Message({
            data: parseFloat(event.target.defaultValue)
        });
        this.im_size_pub.publish(msg);
    }
    

    render() {

        let toggleViewButton = (type) => {
            let input_id = 'view_' + type + '_state';
            let default_class = "mobileToggle";
            let ref = type === 'goal' ? this.viewer_goal_ref : this.viewer_start_ref;
            let input_class = type === 'goal' ? default_class + " toggleGoal" : default_class;
            return (
                <div className="input-wrapper">
                    <span className="input-text">{type}</span>
                    <input type="checkbox" name={type + '_state'} className={input_class}
                        ref={ref} id={input_id} onChange={this.handleViewCallback} />
                    <label htmlFor={input_id}></label>
                </div>
            )  
        }

        let toggleManipulateButton = (type) => {
            let input_id = 'manipulate_' + type + '_state';
            let default_class = "mobileToggle";
            let input_class = type ==='goal' ? default_class + " toggleGoal" : default_class;
            let toggle_state = type === 'goal' ? this.state.toggleGoalManipulation : this.state.toggleStartManipulation;
            return (
                <div className="input-wrapper">
                    <span className="input-text">{type}</span>
                    <input type="checkbox" name={type + '_state'} className={input_class}
                        id={input_id} onChange={this.handleManipulationCallback}
                        checked={toggle_state} />
                    <label htmlFor={input_id}></label>
                </div>  
            )
        }
        
        return (
            <div id="interactive-marker-control">
                <div className="toggler" id="view">View
                    {toggleViewButton('start')}
                    {toggleViewButton('goal')}
                </div>

                {
                    this.state.isViewStartChecked || this.state.isViewGoalChecked ? 
                        <div className="toggler" id="manipulation">Manipulation
                            { this.state.isViewStartChecked ? toggleManipulateButton('start') : <></> }
                            { this.state.isViewGoalChecked ? toggleManipulateButton('goal') : <></> }                  
                        </div> :
                        <></>
                }

                {
                    this.state.toggleStartManipulation || this.state.toggleGoalManipulation ?  
                    <div id="im-size">
                        <h1 id="im-size-title">Size</h1>
                        <div id="im-size-slider">
                            <input type="radio" name="debt-amount" id="extra-small" defaultValue="0.1" onChange={this.handleIMCallback}/>
                            <label htmlFor="extra-small" im-size-value="xs"></label>
                            <input type="radio" name="debt-amount" id="small" defaultValue="0.2" onChange={this.handleIMCallback}/>
                            <label htmlFor="small" im-size-value="s"></label>
                            <input type="radio" name="debt-amount" id="middle" defaultValue="0.3" onChange={this.handleIMCallback}/>
                            <label htmlFor="middle" im-size-value="m"></label>
                            <input type="radio" name="debt-amount" id="large" defaultValue="0.4" onChange={this.handleIMCallback}/>
                            <label htmlFor="large" im-size-value="l"></label>
                            <input type="radio" name="debt-amount" id="extra-large" defaultValue="0.5" onChange={this.handleIMCallback}/>
                            <label htmlFor="extra-large" im-size-value="xl"></label>
                            <div id="im-size-pos"></div>
                        </div>
                    </div> :
                    <></>
                }
            </div>
        )
    }
}