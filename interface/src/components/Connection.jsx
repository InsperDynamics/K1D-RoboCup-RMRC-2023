import { useState, useEffect } from 'react';
import Config from '../scripts/config';
import Alert from 'react-bootstrap/Alert';
import { Container, Row, Col } from 'react-bootstrap';
import Logo from '../img/logo.png';
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
    const [imgdata, setImgdata] = useState(null);
    const [thermaldata, setTermaldata] = useState(null);
    const ipAdress = 'ws://'+Config.ROSBRIDGE_SERVER_IP+':'+Config.ROSBRIDGE_SERVER_PORT

    let autonomousTopic = new window.ROSLIB.Topic({
        ros: ros,
        name: Config.TOPIC_AUTONOMOUS,
        messageType: Config.MSGTYPE_AUTONOMOUS
    });
    let autonomousMsg = new window.ROSLIB.Message({
        data: autonomousOn
    });

    let dexterityTopic = new window.ROSLIB.Topic({
        ros: ros,
        name: Config.TOPIC_DEXTERITY,
        messageType: Config.MSGTYPE_DEXTERITY
    });
    let dexterityMsg = new window.ROSLIB.Message({
        data: dexterityOn
    });
    
    let qrCodeTopic = new window.ROSLIB.Topic({
        ros: ros,
        name: Config.TOPIC_QRCODE,
        messageType: Config.MSGTYPE_QRCODE
    });
    let qrCodeMsg = new window.ROSLIB.Message({
        data: qrCodeOn
    });

    let hazmatTopic = new window.ROSLIB.Topic({
        ros: ros,
        name: Config.TOPIC_HAZMAT,
        messageType: Config.MSGTYPE_HAZMAT
    });
    let hazmatMsg = new window.ROSLIB.Message({
        data: hazmatOn
    });

    let motionTopic = new window.ROSLIB.Topic({
        ros: ros,
        name: Config.TOPIC_MOTION,
        messageType: Config.MSGTYPE_MOTION
    });
    let motionMsg = new window.ROSLIB.Message({
        data: motionOn
    });

    let gasTopic = new window.ROSLIB.Topic({
        ros: ros,
        name: Config.TOPIC_CO2,
        messageType: Config.MSGTYPE_CO2
    });

    let webcamTopic = new window.ROSLIB.Topic({
        ros: ros,
        name: Config.TOPIC_CAMERA,
        messageType: Config.MSGTYPE_CAMERA
    });

    let thermalTopic = new window.ROSLIB.Topic({
        ros: ros,
        name: Config.TOPIC_TEMPERATURE,
        messageType: Config.MSGTYPE_TEMPERATURE
    });

    
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

    function pubAutonomous(msg, data, topic) {
        setAutonomousOn(!data);
        msg.data = !data;
        topic.publish(msg);
    }

    function pubDexterity(msg, data, topic) {
        setDexterityOn(!data);
        msg.data = !data;
        topic.publish(msg);
    }

    function pubQrCode(msg, data, topic) {
        setQrCodeOn(!data);
        msg.data = !data;
        topic.publish(msg);
    }

    function pubHazmat(msg, data, topic) {
        setHazmatOn(!data);
        msg.data = !data;
        topic.publish(msg);
    }

    function pubMotion(msg, data, topic) {
        setMotionOn(!data);
        msg.data = !data;
        topic.publish(msg);
    }

    gasTopic.subscribe((message) => {
        setPpm(message.data);
    });

    webcamTopic.subscribe(function(message) {
        setImgdata(message.data);
    });

    thermalTopic.subscribe((message) => {
        setTermaldata(message.data);
    });


    init_connection();

    return (
        <Container fluid style={{margin: "0px"}}>
            <Row>
                <Col>
                    <img src={Logo} alt="Insper Dynamics" width={150} height={150}/>
                </Col>
                <Col className='mt-5'>
                    <Alert className="text-center" letiant={isConnected? "success": "danger"}>
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
                    <img src={imgdata != null ? `data:image/jpeg;base64,` + imgdata: `https://via.placeholder.com/${1280}x${480}`} width={1280} height={480} alt="camera"/>
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
                    <button className="btn btn-primary" style={{backgroundColor: (autonomousOn? "green": "red")}} onClick={() => pubAutonomous(autonomousMsg, autonomousOn, autonomousTopic)}>
                        {autonomousOn? "Autonomous On": "Autonomous Off"}
                    </button>
                </Col>
                <Col>
                    <button className="btn btn-primary" style={{backgroundColor: (dexterityOn? "green": "red")}} onClick={() => pubDexterity(dexterityMsg, dexterityOn, dexterityTopic)}>
                        {dexterityOn? "Dexterity On": "Dexterity Off"}
                    </button>
                </Col>
                <Col>
                    <button className="btn btn-primary" style={{backgroundColor: (qrCodeOn? "green": "red")}} onClick={() => pubQrCode(qrCodeMsg, qrCodeOn, qrCodeTopic)}>
                        {qrCodeOn? "QR Code On": "QR Code Off"}
                    </button>
                </Col>
                <Col>
                    <button className="btn btn-primary" style={{backgroundColor: (hazmatOn? "green": "red")}} onClick={() => pubHazmat(hazmatMsg, hazmatOn, hazmatTopic)}>
                        {hazmatOn? "Hazmat On": "Hazmat Off"}
                    </button>
                </Col>
                <Col>
                    <button className="btn btn-primary" style={{backgroundColor: (motionOn? "green": "red")}} onClick={() => pubMotion(motionMsg, motionOn, motionTopic)}>
                        {motionOn? "Motion On": "Motion Off"}
                    </button>
                </Col>
                <Col>
                    <div>
                        <img src={thermaldata != null ? `data:image/jpeg;base64,` + thermaldata: `https://via.placeholder.com/${160}x${160}`} width={160} height={160} alt="camera"/>
                    </div>
                    <p className="" style={{color: "#e66111", fontSize: "15px", marginLeft: "29px"}}>Thermal Cam</p>
                </Col>
                <Col>
                    <h4 className="text-left m-3" style={{color: "#e66111"}}>{ppm != null ? ppm : "Loading gas sensor..."}{ppm && "PPM"}</h4>
                </Col>
            </Row>
        </Container>
    );
}

export default Connection;
