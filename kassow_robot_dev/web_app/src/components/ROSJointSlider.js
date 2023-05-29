import { Component, createRef, Fragment } from "react";
import ROSLIB from "roslib";
import { Context } from "./ContextProvider";

export default class ROSJointSlider extends Component {
    static contextType = Context;

    constructor(props) {
        super(props);
        this.link_group = props.link_group;
        this.joint_names = null;
        this.ros = props.ros;
        this.selected_group = [Object.keys(this.link_group)[0]];
        this.joint_names_param = new ROSLIB.Param({
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

        this.state = {
            isJointNamesLoaded : false
        }
    }
    async componentDidMount() {
        await new Promise( (resolve, reject) => {
            this.joint_names_param.get( (value) => {
                this.joint_names = value;
            })
            let timerID = setInterval( () => {
                if(this.joint_names !== null){
                    clearInterval(timerID);
                    resolve();
                }
            }, 100);
            setTimeout( () => {
                clearInterval(timerID);
                // TODO: Set a state in react which will give an alert that something has gone wrong.
                reject(new Error('Failed to fetch from Joint Names ROSParam.'));
            }, 10000)
        });
        this.context.state.selected_group = this.selected_group;
        this.setState({ isJointNamesLoaded : true });
    }

    componentWillUnmount() {
        this.setState({ isJointNamesLoaded : false })
    }

    componentDidUpdate() {
        this.createJointPositionMsg(1, true);
    }

    createJointPositionMsg = (type, plan_only) => {
        let positions = [];
        let start_positions = [];
        let goal_positions = [];
        let dims = [];
        Array.from(this.props.joint_group_slider.current.children).map( (group) => {
            Array.from(group.children).map( (joint_html) => {
                if (joint_html.tagName === 'LABEL') {
                    let dim = new ROSLIB.Message({
                        label: joint_html.htmlFor,
                        size: joint_html.htmlFor.length,
                        stride: joint_html.htmlFor.length
                    });
                    dims.push(dim);
                    if (type === 0) {
                        this.props.start_joint_states.name.map( (joint, index) => {
                            if (joint === dim.label) {
                                start_positions.push(this.props.start_joint_states.position[index]);
                                goal_positions.push(this.props.goal_joint_states.position[index]);
                            }
                        })
                    }
                    else {
                        positions.push(parseFloat(joint_html.nextElementSibling.value))
                    }
                }
            })
        })
        let msg;
        this.selected_group.map( (group) => {
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
                    group_name: group
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
            });
        return msg;
    }

    handleGroupChange = (event) => {
        this.selected_group = Array.from(event.target.selectedOptions).map( option => option.value);
        this.context.state.selected_group = this.selected_group;
        this.createJointPositionMsg(1, true);
    }

    handleJointChange = (event) => {
        let msg = this.createJointPositionMsg(1, true);
        if (this.context.state.isStartManipulateActive) {
            this.start_pub.publish(msg);
        }
        else if (this.context.state.isGoalManipulateActive){
            this.goal_pub.publish(msg);
        }
    }

    render() {
        let group_name = Object.keys(this.link_group).map((key) => {
            return <option key={key} id={key}>{key}</option>
        });
    
        let joint_slider_view = this.selected_group.map( (group, index) => {
            return (
                this.state.isJointNamesLoaded ? 
                <div key={index} id={group}>
                    {group}
                    {Object.values(this.joint_names.names).map( (name, index) => {
                        let isJointPresent = this.link_group[group].includes(name);
                        return ( isJointPresent ? 
                                <Fragment key={index}>
                                    <label htmlFor={name}>{name}</label> 
                                    <input type="range" name={name} id={name} defaultValue="0"
                                        max={this.joint_names[name].max} min={this.joint_names[name].min}
                                        step="0.000001" onChange={this.handleJointChange}>
                                    </input>
                                </Fragment>
                            :
                            <Fragment key={index}></Fragment>
                            )
                        })
                    }
                </div>
                :
                <div key={index}>Loading joint names...</div>
            )   
        })

        return (
            <>
                <select id="group" name="group" onChange={this.handleGroupChange} multiple>
                    {group_name}
                </select>
                <div id="slider-pane" ref={this.props.joint_group_slider}>
                    {joint_slider_view}
                </div>
            </>
        )
    }
}