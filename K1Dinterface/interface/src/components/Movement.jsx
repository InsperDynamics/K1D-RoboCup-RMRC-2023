// import { useState } from 'react';
// import Config from '../scripts/config';

// function Movement() {
//   const [ros, setRos] = useState(new window.ROSLIB.Ros());
  
//   function init_connection() {
//     ros.on('connection', () => {
//         setIsConnected(true);
//     });
//     ros.on('close', () => {
//         setIsConnected(false);
//         // try to reconnect every 5 seconds
//         setTimeout(() => {
//             try {
//                 ros.connect(ipAdress);
//             } catch (error) {
//                 console.log("connection error");
//             }
//         }, Config.RECONNECTION_TIMER);
//     });
//     try {
//         ros.connect(ipAdress);
//     } catch (error) {
//         console.log("connection error");
//     }
// }

//   function onAutonomous() {
//     var autonomousTopic = new window.ROSLIB.Topic({
//       ros: ros,
//       name: Config.TOPIC_AUTONOMOUS,
//       messageType: Config.MSGTYPE_AUTONOMOUS
//     });
//     var autonomousMsg = new window.ROSLIB.Message({
//       bool: true
//     });
//     autonomousTopic.publish(autonomousMsg);
//   }

//   return (
//     <></>
//   );

// }

// export default Movement;
