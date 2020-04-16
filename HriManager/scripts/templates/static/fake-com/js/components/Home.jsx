import React, { Component } from 'react';
// import SpeakableButton from './SpeakableButton';
import App from './../robocup_pepper-hri_js/src'
import socketIOClient from "socket.io-client";

export default class Home extends Component {

      constructor() {
        super();
        this.state = {
          response: false,
          endpoint: "http://127.0.0.1:5000"
        };
      }
    //   componentDidMount() {
    //     const { endpoint } = this.state;
    //     const socket = socketIOClient(endpoint);
    //     socket.on('connect', function () { socket.emit('my event', { data: 'I\'m connected!' }) });
    //   }
    render() {
       return (
        // <body>
        // <script src="//cdnjs.cloudflare.com/ajax/libs/socket.io/2.2.0/socket.io.js"
        //         integrity="sha256-yr4fRk/GU1ehYJPAs8P4JlTgu0Hdsp4ZKrx8bDEDC3I=" crossorigin="anonymous"></script>
        //     <script type="text/javascript" charset="utf-8">

        //         $(document).ready(function() {
        //         var socket = io.connected('http:127.0.0.1:5000');

        //         socket.on('connect', function () {
        //             socket.emit('my event', { data: 'I\'m connected!' })
        //         });
        //     });
        //     </script>
        // </body>
        //   <h1>Hello Reactttttt!</h1>
          <App/>
       )
    }
}