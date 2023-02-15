import { useState } from 'react';
import Config from '../scripts/config';
import Alert from 'react-bootstrap/Alert';
import { Container, Row, Col} from 'react-bootstrap';

function Connection() {
    const [isConnected, setIsConnected] = useState(false);
    const [ros, setRos] = useState(new window.ROSLIB.Ros());
    const [autonomousOn, setAutonomousOn] = useState(false);
    const [dexterityOn, setDexterityOn] = useState(false);
    const [qrCodeOn, setQrCodeOn] = useState(false);
    const [hazmatOn, setHazmatOn] = useState(false);
    const [motionOn, setMotionOn] = useState(false);
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

    function onAutonomous() {
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
    function onDexterity() {
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
    function onQrCode() {
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
    function onHazmat() {
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
    function onMotion() {
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

    init_connection();

    return (
        <Container>
            <Row>
                <Col md={{ span: 6, offset: 3 }}>
                    <Alert className="text-center m-3" variant={isConnected? "success": "danger"}>
                        {isConnected? "Robot Connected": "Robot Disconnected"}
                    </Alert>
                </Col>
            </Row>
            <Row>
                <Col>
                    <h1 className="text-left m-3" style={{color: "#e66111"}}>Camera</h1>
                </Col>
                <Col>
                    <h1 className="text-right m-3" style={{color: "#e66111"}}>Sensors</h1>
                </Col>
            </Row>
            <Row>

            </Row>
            <Row>
                <h1 className="text-center m-3" style={{color: "#e66111"}}>Switch Modes</h1>
            </Row>
            <Row>
                <Col>
                    <button className="btn btn-primary" style={{backgroundColor: (autonomousOn? "green": "red")}} onClick={() => onAutonomous()}>
                        {autonomousOn? "Autonomous On": "Autonomous Off"}
                    </button>
                </Col>
                <Col>
                    <button className="btn btn-primary" style={{backgroundColor: (dexterityOn? "green": "red")}} onClick={() => onDexterity()}>
                        {dexterityOn? "Dexterity On": "Dexterity Off"}
                    </button>
                </Col>
                <Col>
                    <button className="btn btn-primary" style={{backgroundColor: (qrCodeOn? "green": "red")}} onClick={() => onQrCode()}>
                        {qrCodeOn? "QR Code On": "QR Code Off"}
                    </button>
                </Col>
                <Col>
                    <button className="btn btn-primary" style={{backgroundColor: (hazmatOn? "green": "red")}} onClick={() => onHazmat()}>
                        {hazmatOn? "Hazmat On": "Hazmat Off"}
                    </button>
                </Col>
                <Col>
                    <button className="btn btn-primary" style={{backgroundColor: (motionOn? "green": "red")}} onClick={() => onMotion()}>
                        {motionOn? "Motion On": "Motion Off"}
                    </button>
                </Col>
            </Row>
        </Container>
    );
}

export default Connection;
