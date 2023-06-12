import React from 'react';
import AutoRos from './AutoRos';
import { ContextProvider } from './ContextProvider';
import ROSViewer from './ROSViewer';
import ROSInitUrdf from './ROSInitUrdf';
import ROSControlTopic from './ROSControlTopic';

export default class ROSConnect extends React.Component {
    constructor(props) {

        super(props);
        // Connect to ROS.
        this.autoROS = new AutoRos({
            reconnectTimeOut: 5000
        });
        const url = 'ws://10.229.199.16:9090';
        this.autoROS.connect(url);
        this.ros = this.autoROS.ros;
        this.state = {
            isROSConnected: false,
        }
    }

    componentDidMount () {
        setInterval( () => {
            this.setState({ isROSConnected: this.ros.isConnected});
        }, 100);
    }
    
    render() {
        return (
            <>
                {
                    this.state.isROSConnected ?
                    <ContextProvider>
                        <ROSViewer />
                        <ROSInitUrdf ros={this.ros} />
                        <ROSControlTopic ros={this.ros}/>
                    </ContextProvider>
                    :
                    <p>Connecting to ROS Web Socket</p>
                }
            </>
        )
    }
}