import { Component } from "react";
import { Context } from "./ContextProvider";
import { Message, Topic } from "roslib";
import '../styles/IMSize.css';


export default class ROSIMSize extends Component {
    static contextType = Context;

    constructor(props) {
        super(props);
        this.ros = props.ros;
        this.im_size_pub = new Topic({
            ros: this.ros,
            name: '/im_size/update',
            messageType: 'std_msgs/Float32'
        });
        this.state = {
            backgroundColor : 'red',
        }
    }

    handleIMCallback = (event) => {
        let msg = new Message({
            data: parseFloat(event.target.defaultValue)
        });
        this.im_size_pub.publish(msg);

        if (event.target.checked) {
            if (this.context.state.isStartManipulateActive) {
                this.setState({ backgroundColor : '#2ecc71' })
            }
            else if (this.context.state.isGoalManipulateActive) {
                this.setState({ backgroundColor : 'red' })
            }
        }
    }


    componentDidUpdate() {
        this.context.state.viewer.selectableObjects.children.map( (im) => {
            if (im.name === 'start') {
                im.visible = this.context.state.isStartManipulateActive
            }
            else if (im.name === 'goal') {
                im.visible = this.context.state.isGoalManipulateActive
            }
        })
    }

    componentWillUnmount() {
        this.im_size_pub.unsubscribe();
        this.setState({ backgroundColor : 'red' })
    }

    render() {
        return (
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
                    <div id="im-size-pos" style={{ backgroundColor: this.state.backgroundColor }}></div>
                </div>
            </div>
        ) 
    }
}
