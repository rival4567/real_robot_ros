import { Component, createRef } from 'react';
import { Viewer, MeshResource, Grid, makeColorMaterial } from '../ros3djs/src-esm';
import { Context } from './ContextProvider';


export default class ROSViewer extends Component {
    static contextType = Context;

    constructor(props) {
        super(props);
        this.viewerRef = createRef();
        this.viewer = null;
        this.resizeObserver = null;
    }

    componentDidMount() {
        this.initViewer();
        this.initResizeObserver();
        this.context.state.viewer = this.viewer;
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
        this.viewer = new Viewer({
            divID : this.viewerRef.current.id,
            width : width,
            height : height,
            antialias : true,
            intensity : 0.09,
            background : '#415A77',
            alpha: 1.0
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
    
        this.viewer.scene.castShadow = true
    
        const drawer = new MeshResource({
            resource: 'ablagebox.dae',
            path : '/meshes/',
            warnings : true,
            material : new makeColorMaterial(0.21176470588, 0.27058823529, 0.30980392156, 1.0)
        });
    
        const pedestal = new MeshResource({
            resource: 'Roboter_sockel.dae',
            path : '/meshes/',
            warnings : true,
            material : new makeColorMaterial(0.21176470588, 0.27058823529, 0.30980392156, 0.5)
        });
    
        drawer.position.set(0, 0, -0.41)
        drawer.scale.set(0.3, 0.3, 0.3)
        drawer.castShadow = true
        drawer.receiveShadow = true
    
        pedestal.position.set(-0.2, 0.225, -0.41)
        pedestal.scale.set(0.0025, 0.0025, 0.0025)
        pedestal.castShadow = true
        pedestal.receiveShadow = true
    
        this.viewer.addObject(drawer);
        this.viewer.addObject(pedestal);
    
        // Add a grid.
        this.viewer.addObject(new Grid());
        this.viewer.scene.visible = true;
    }

    initResizeObserver() {
        const targetNode = this.viewerRef.current;
        this.resizeObserver = new ResizeObserver((entries) => {
            const { width, height } = entries[0].contentRect;
            this.viewer.resize(width, height);
        });
        this.resizeObserver.observe(targetNode);
    }
    render () {
        return (
            <div className="viewer">
                <div id="rosViewer" style={{ width: '100%', height: '100%' }} ref={this.viewerRef}></div>
            </div>
        )
    }
}