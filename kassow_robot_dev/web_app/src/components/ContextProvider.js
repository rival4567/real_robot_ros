import { Component, createContext } from "react";

export const Context = createContext();  //exporting context object

export class ContextProvider extends Component {
    state = {
        viewer: null,
        fixed_frame: null,
        link_group: null,
        selected_group: null,
        joint_value: 0,
        isStartManipulateActive: false,
        isGoalManipulateActive: false,
    }
    render() {
        return (
            <Context.Provider value={
                {   state: this.state,
                    setViewer: (value) => this.setState({
                        viewer: value }),
                    setJointValue: (value) => this.setState({
                        joint_value: value
                    })
                }
            }>
            {/* this indicates that the global store is accessible to
             all the child tags with ContextProvider as Parent */}
            {this.props.children} 
            </Context.Provider>
        )
    }
}