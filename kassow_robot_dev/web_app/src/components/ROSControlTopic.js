import { Component, createRef } from "react";
import { Context } from "./ContextProvider";
import ROSLIB from "roslib";
import ROSJointSlider from './ROSJointSlider';
import ROSInteractiveMarker from "./ROSInteractiveMarker";
import ROSButton from "./ROSButton";

export default class ROSControlTopic extends Component {
    static contextType = Context;

    constructor(props) {
        super(props);
        this.ros = props.ros;
        this.link_group = null;
        this.joint_states = null;
        this.start_joint_states = null;
        this.goal_joint_states = null;
        this.current_group = null;
        this.state = {
            isLinkGroupLoaded : false
        }

        this.joint_group_slider = createRef();

        this.message_stock = [];

        this.link_group_param = new ROSLIB.Param({
            ros: this.ros,
            name: '/link_group/'
        });        

        this.plan_listener = new ROSLIB.Topic({
            ros: this.ros,
            name: '/stock_joint_position',
            messageType: 'std_msgs/Float64MultiArray'
        });

        // Setup listener
        this.joint_listener = new ROSLIB.Topic({
            ros : this.ros,
            name : '/joint_states',
            messageType : 'sensor_msgs/JointState'
        });
        this.start_listener = new ROSLIB.Topic({
            ros : this.ros,
            name : '/start_joint_states',
            messageType : 'sensor_msgs/JointState'
        });
        this.goal_listener = new ROSLIB.Topic({
            ros : this.ros,
            name : '/goal_joint_states',
            messageType : 'sensor_msgs/JointState'
        });

        // Publisher
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

    }

    async componentDidMount() {
        await new Promise( (resolve, reject) => {
            this.plan_listener.subscribe( (message) => {
                this.message_stock.push(message);
            });
            this.joint_listener.subscribe( (message) => {
                this.joint_states = message;
            });        
            this.start_listener.subscribe( (message) => {
                this.start_joint_states = message;
            });
            this.goal_listener.subscribe( (message) => {
                this.goal_joint_states = message;
            });
            this.link_group_param.get( (value) => {
                this.link_group = value;
            });
            let timerID = setInterval( () => {
                if(this.link_group !== null && this.joint_listener !== null
                    && this.start_joint_states !== null && this.goal_joint_states !== null){
                    clearInterval(timerID);
                    resolve();
                }
            }, 100);
            // Rejection after 10seconds
            setTimeout(() => {
                clearInterval(timerID);
                // TODO: Set a state in react which will give an alert that something has gone wrong.
                reject(new Error('Failed to fetch from Link Group ROSParam.'));
            }, 10000)
        });
        this.setState({ isLinkGroupLoaded: true });
    }

    componentWillUnmount() {
        this.plan_listener.unsubscribe();
        this.joint_listener.unsubscribe();
        this.goal_listener.unsubscribe();
        this.start_pub.unsubscribe();
        this.goal_pub.unsubscribe();
        this.setState({ isLinkGroupLoaded: false });
    }

    render() {
        return (
            <div className='control-panel'>
                {
                    this.state.isLinkGroupLoaded ? 
                        <>
                            <ROSJointSlider
                                ros={this.ros}
                                link_group={this.link_group}
                                start_pub={this.start_pub}
                                goal_pub={this.goal_pub}
                                joint_group_slider={this.joint_group_slider}
                                start_joint_states={this.start_joint_states}
                                goal_joint_states={this.goal_joint_states}
                            />
                            <ROSInteractiveMarker 
                                ros={this.ros}
                                start_joint_states={this.start_joint_states}
                                goal_joint_states={this.goal_joint_states}
                            />
                            <ROSButton
                                ros={this.ros}
                                joint_states={this.joint_states}
                                start_pub={this.start_pub}
                                goal_pub={this.goal_pub}
                                joint_group_slider={this.joint_group_slider}
                                message_stock={this.message_stock}
                                start_joint_states={this.start_joint_states}
                                goal_joint_states={this.goal_joint_states}
                            />
                        </>
                        : <div>Loading JointSlider</div>
                }
            </div>
        )
    }
}