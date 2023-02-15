import { useState } from 'react';
import Config from '../scripts/config';
import Alert from 'react-bootstrap/Alert';
import { Container, Row, Col } from 'react-bootstrap';
import Logo from './../img/logo.png';
import RosImg from './RosImg';

// resolução do site: 1920px x 920px

function Connection() {
    const [isConnected, setIsConnected] = useState(false);
    const [ros, setRos] = useState(new window.ROSLIB.Ros());
    
    const [autonomousOn, setAutonomousOn] = useState(false);
    const [dexterityOn, setDexterityOn] = useState(false);
    const [qrCodeOn, setQrCodeOn] = useState(false);
    const [hazmatOn, setHazmatOn] = useState(false);
    const [motionOn, setMotionOn] = useState(false);
    
    const [ppm, setPpm] = useState(null);
    const ipAdress = 'ws://'+Config.ROSBRIDGE_SERVER_IP+':'+Config.ROSBRIDGE_SERVER_PORT
    
    
    function init_connection() {
        ros.on('connection', () => {
            setIsConnected(true);
        });
        ros.on('close', () => {
            setIsConnected(false);
            // try to reconnect every 5 seconds
            setTimeout(() => {
                try {
                    ros.connect(ipAdress);
                } catch (error) {
                    console.log("connection error");
                }
            }, Config.RECONNECTION_TIMER);
        });
        try {
            ros.connect(ipAdress);
        } catch (error) {
            console.log("connection error");
        }
    }

    function pubAutonomous() {
        setAutonomousOn(!autonomousOn);
        var autonomousTopic = new window.ROSLIB.Topic({
            ros: ros,
            name: Config.TOPIC_AUTONOMOUS,
            messageType: Config.MSGTYPE_AUTONOMOUS
        });
        var autonomousMsg = new window.ROSLIB.Message({
            data: autonomousOn
        });
        autonomousTopic.publish(autonomousMsg);
    }
    function pubDexterity() {
        setDexterityOn(!dexterityOn);
        var dexterityTopic = new window.ROSLIB.Topic({
            ros: ros,
            name: Config.TOPIC_DEXTERITY,
            messageType: Config.MSGTYPE_DEXTERITY
        });
        var dexterityMsg = new window.ROSLIB.Message({
            data: dexterityOn
        });
        dexterityTopic.publish(dexterityMsg);
    }
    function pubQrCode() {
        setQrCodeOn(!qrCodeOn);
        var qrCodeTopic = new window.ROSLIB.Topic({
            ros: ros,
            name: Config.TOPIC_QRCODE,
            messageType: Config.MSGTYPE_QRCODE
        });
        var qrCodeMsg = new window.ROSLIB.Message({
            data: qrCodeOn
        });
        qrCodeTopic.publish(qrCodeMsg);
    }
    function pubHazmat() {
        setHazmatOn(!hazmatOn);
        var hazmatTopic = new window.ROSLIB.Topic({
            ros: ros,
            name: Config.TOPIC_HAZMAT,
            messageType: Config.MSGTYPE_HAZMAT
        });
        var hazmatMsg = new window.ROSLIB.Message({
            data: hazmatOn
        });
        hazmatTopic.publish(hazmatMsg);
    }
    function pubMotion() {
        setMotionOn(!motionOn);
        var motionTopic = new window.ROSLIB.Topic({
            ros: ros,
            name: Config.TOPIC_MOTION,
            messageType: Config.MSGTYPE_MOTION
        });
        var motionMsg = new window.ROSLIB.Message({
            data: motionOn
        });
        motionTopic.publish(motionMsg);
    }
    function subGas() {
        var gasTopic = new window.ROSLIB.Topic({
            ros: ros,
            name: Config.TOPIC_CO2,
            messageType: Config.MSGTYPE_CO2
        });
        gasTopic.subscribe((message) => {
            // document.getElementById('thermalcam_img').src = "data:image/jpg;base64," + message.data;
            gasTopic.unsubscribe();
            setPpm(message.data);
        });
    }

    init_connection();
    subGas();

    return (
        <Container fluid style={{margin: "0px"}}>
            <Row>
                <Col>
                    <img src={Logo} alt="Insper Dynamics" width={150} height={150}/>
                </Col>
                <Col className='mt-5'>
                    <Alert className="text-center" variant={isConnected? "success": "danger"}>
                        {isConnected? "Robot Connected": "Robot Disconnected"}
                    </Alert>
                </Col>
                <Col className='mt-5'>
                    <h1 className="text-right" style={{color: "#e66111"}}>K1D Control Panel</h1>
                </Col>
            </Row>
            <h1 className="text-left" style={{color: "#e66111"}}>Camera</h1>
            <Row>
                <Col style={{display: "flex", justifyContent: "center"}}>
                    <RosImg style={{justifySelf: "center"}} ros={ros} topic={Config.TOPIC_CAMERA} msg={Config.MSGTYPE_CAMERA} width={1280} height={480} />
                </Col>
            </Row>
            
            <Row>
                <Col>
                    <h1 className="text-left mt-3" style={{color: "#e66111"}}>Switch Modes</h1>
                </Col>
                <Col md={4} style={{paddingLeft: "75px"}}>
                    <h1 className="text-left mt-3" style={{color: "#e66111"}}>Sensors</h1>
                </Col>
            </Row>
            <Row>
                <Col>
                    <button className="btn btn-primary" style={{backgroundColor: (autonomousOn? "green": "red")}} onClick={() => pubAutonomous()}>
                        {autonomousOn? "Autonomous On": "Autonomous Off"}
                    </button>
                </Col>
                <Col>
                    <button className="btn btn-primary" style={{backgroundColor: (dexterityOn? "green": "red")}} onClick={() => pubDexterity()}>
                        {dexterityOn? "Dexterity On": "Dexterity Off"}
                    </button>
                </Col>
                <Col>
                    <button className="btn btn-primary" style={{backgroundColor: (qrCodeOn? "green": "red")}} onClick={() => pubQrCode()}>
                        {qrCodeOn? "QR Code On": "QR Code Off"}
                    </button>
                </Col>
                <Col>
                    <button className="btn btn-primary" style={{backgroundColor: (hazmatOn? "green": "red")}} onClick={() => pubHazmat()}>
                        {hazmatOn? "Hazmat On": "Hazmat Off"}
                    </button>
                </Col>
                <Col>
                    <button className="btn btn-primary" style={{backgroundColor: (motionOn? "green": "red")}} onClick={() => pubMotion()}>
                        {motionOn? "Motion On": "Motion Off"}
                    </button>
                </Col>
                <Col>
                    <div>
                        <RosImg ros={ros} topic={Config.TOPIC_TEMPERATURE} msg={Config.MSGTYPE_TEMPERATURE} width={160} height={160} />
                    </div>
                    <p className="" style={{color: "#e66111", fontSize: "15px", marginLeft: "29px"}}>Thermal Cam</p>
                </Col>
                <Col>
                    <h4 className="text-left m-3" style={{color: "#e66111"}}>{ppm ? ppm : "Loading gas sensor..."}{ppm && "PPM"}</h4>
                </Col>
            </Row>
        </Container>
    );
}

export default Connection;
