import { Component } from "react";
import { Context } from "./ContextProvider";
import { Message } from "roslib";

export default class ROSButton extends Component {
    static contextType = Context;

    constructor(props) {
        super(props);
        this.joint_states = props.joint_states;
        this.start_pub = props.start_pub;
        this.goal_pub = props.goal_pub;
        this.joint_group_slider = props.joint_group_slider;
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


    render() {
        return (
            <>
                <button id="reset" onPointerDown={this.handleReset}>Reset</button>
                <button id="preview">Preview</button>
                <button id="waypoint">Add waypoint</button>
                <button id="plan">Plan</button>
                <button id="execute">Execute</button>
                <button id="moveit">Plan & Execute</button>
            </>
        )
    }
}
