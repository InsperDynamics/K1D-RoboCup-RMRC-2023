// Connecting to ROS
var ros = new ROSLIB.Ros({
    url : 'ws://localhost:9090'
});

ros.on('connection', function() {
    console.log('Connected to websocket server.');
});

ros.on('error', function(error) {
    console.log('Error connecting to websocket server: ', error);
});

ros.on('close', function() {
    console.log('Connection to websocket server closed.');
});

// Declaring Topics
var webcam = new ROSLIB.Topic({
    ros : ros,
    name : '/webcam',
    messageType : 'sensor_msgs/Image'
});
var thermalcam = new ROSLIB.Topic({
    ros : ros,
    name : '/thermalcam',
    messageType : 'sensor_msgs/Image'
});
var gas = new ROSLIB.Topic({
    ros : ros,
    name : '/gas',
    messageType : 'std_msgs/UInt16'
});
var autonomous_mode = new ROSLIB.Topic({
    ros : ros,
    name : '/autonomous_mode',
    messageType : 'std_msgs/Bool'
});
var dexterity_mode = new ROSLIB.Topic({
    ros : ros,
    name : '/dexterity_mode',
    messageType : 'std_msgs/Bool'
});
var qr_detection = new ROSLIB.Topic({
    ros : ros,
    name : '/qr_detection',
    messageType : 'std_msgs/Bool'
});
var hazmat_detection = new ROSLIB.Topic({
    ros : ros,
    name : '/hazmat_detection',
    messageType : 'std_msgs/Bool'
});
var motion_detection = new ROSLIB.Topic({
    ros : ros,
    name : '/motion_detection',
    messageType : 'std_msgs/Bool'
});

// Subscribing to Topics
webcam.subscribe(function(message) {
    console.log('Received message on ' + listener.name + ': ' + message.data);
});
thermalcam.subscribe(function(message) {
    console.log('Received message on ' + listener.name + ': ' + message.data);
});
gas.subscribe(function(message) {
    console.log('Received message on ' + listener.name + ': ' + message.data);
});

// Publishing to Topics
