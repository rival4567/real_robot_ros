import { Component } from "react";
import { Context } from "./ContextProvider";
import { Message, Topic } from "roslib";
import '../styles/Button.css';

export default class ROSButton extends Component {
    static contextType = Context;

    constructor(props) {
        super(props);
        this.joint_states = props.joint_states;
        this.start_pub = props.start_pub;
        this.goal_pub = props.goal_pub;
        this.joint_group_slider = props.joint_group_slider;
        this.moveit_pub = new Topic({
            ros: this.props.ros,
            name: '/moveit_joint',
            messageType: 'rwt_moveit/MoveGroupPlan'
        });
    }

    createJointPositionMessage = (plan_only) => { 
        let start_positions = [];
        let goal_positions = [];
        let dims = [];
        Array.from(this.props.joint_group_slider.current.children).map( (group) => {
            Array.from(group.children).map( (joint_html) => {
                if (joint_html.tagName === 'LABEL') {
                    let dim = new Message({
                        label: joint_html.htmlFor,
                        size: joint_html.htmlFor.length,
                        stride: joint_html.htmlFor.length
                    });
                    dims.push(dim);
                    this.props.start_joint_states.name.map( (joint, index) => {
                        if (joint === dim.label) {
                            start_positions.push(this.props.start_joint_states.position[index]);
                            goal_positions.push(this.props.goal_joint_states.position[index]);
                        }
                    })
                }
            })
        })
        let message;
        if (this.context.state.selected_group) {
            this.context.state.selected_group.map( (group) => {
                message = new Message({
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
            });
        }
        return message;
    }

    handleReset = (event) => {
        let positions = [];
        let dims = [];

        Array.from(this.joint_group_slider.current.children).map( (group) => {
            Array.from(group.children).map( (joint_html) => {
                if (joint_html.tagName === 'LABEL') {
                    let dim = new Message({
                        label: joint_html.htmlFor,
                        size: joint_html.htmlFor.length,
                        stride: joint_html.htmlFor.length
                    });
                    dims.push(dim);
                    this.joint_states.name.map( (name, index) => {
                        if (name === dim.label) {
                            positions.push(this.joint_states.position[index])
                        }
                    })
                }
            })
        }) 

        let message;
        message = new Message({
            layout: {
                dim: dims,
                data_offset: 0
            },
            data: positions
        });

        if (this.context.state.isStartManipulateActive) {
            this.start_pub.publish(message);
        }
        else if (this.context.state.isGoalManipulateActive) {
            this.goal_pub.publish(message);
        }
        
        // this.display_planned_path.sn.unsubscribeTf();
        // this.display_planned_path.rootObject.visible = false;
    }

    handlePreview = (event) => {
        if(typeof this.props.message_stock !== 'undefined') {
            let idx = 0;
            let temp_start_joint_states = this.start_joint_states;
            console.log('fjakf', this.props.message_stock[0])
            let timer = setInterval( () => {
                this.start_pub.publish(this.props.message_stock[idx]);
                idx++;
                if(idx === this.props.message_stock.length) {

                    let positions = [];
                    let dims = [];

                    Array.from(this.joint_group_slider.current.children).map( (group) => {
                        Array.from(group.children).map( (joint_html) => {
                            if (joint_html.tagName === 'LABEL') {
                                let dim = new Message({
                                    label: joint_html.htmlFor,
                                    size: joint_html.htmlFor.length,
                                    stride: joint_html.htmlFor.length
                                });
                                dims.push(dim);
                                temp_start_joint_states.name.map( (name, index) => {
                                    if (name === dim.label) {
                                        positions.push(temp_start_joint_states.position[index])
                                    }
                                })
                            }
                        })
                    }) 

                    let message;
                    message = new Message({
                        layout: {
                            dim: dims,
                            data_offset: 0
                        },
                        data: positions
                    });
                    this.start_pub.publish(message);
                    clearInterval(timer);
                }
            }, 100);
        }
    }

    handleMoveit = (event) => {
        let message = this.createJointPositionMessage(false);
        this.moveit_pub.publish(message);
    }

    handlePlan = (event) => {
        let message = this.createJointPositionMessage(0, true);
        this.moveit_pub.publish(message);
    }

    // document.querySelector("button#execute").addEventListener("pointerdown", () => {
    //     if(typeof this.message_stock !== 'undefined') {
    //         let sim_mode = new ROSLIB.Param({
    //             ros: this.ros,
    //             name: '/sim_mode'
    //         });
    //         sim_mode.get( (value) => {
    //             if (value !== true) {
    //                 let timer = setInterval( () => {
    //                     console.log("execute trajectory", this.message_stock)
    //                     if(this.message_stock.length === 0) {
    //                         clearInterval(timer);
    //                     }
    //                     else {
    //                         this.joint_pub.publish(this.message_stock.shift());
    //                     }
    //                 }, 100);
    //             }
    //             else {
    //                 let msg = new ROSLIB.Message({
    //                 });
    //                 this.execute_pub.publish(msg);
    //             }                
    //         });
    //     }
    // });


    render() {
        return (
            <>
                <button id="reset" className='custom-btn btn-5' onPointerDown={this.handleReset}>Reset</button>
                <button id="preview" className='custom-btn btn-5' onPointerDown={this.handlePreview}>Preview</button>
                <button id="waypoint" className='custom-btn btn-5'>Add waypoint</button>
                <button id="plan" className='custom-btn btn-5'onPointerDown={this.handlePlan}>Plan</button>
                <button id="execute" className='custom-btn btn-5' onPointerDown={this.handleExecute}>Execute</button>
                <button id="moveit" className='custom-btn btn-5' onPointerDown={this.handleMoveit}>Plan & Execute</button>
            </>
        )
    }
}
