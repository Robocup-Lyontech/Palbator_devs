import React, { Component } from 'react';
import socketIOClient from "socket.io-client";
// import {connect} from 'react-redux';
import PropTypes from "prop-types";

// function mapStateToProps(state) {
// 	return {};
// }

export default class SpeakableButton extends React.Component {
    constructor(props) {
        super(props);
        this.state = {
            response: false,
            socket: socketIOClient("http://127.0.0.1:5000"),
        };
        // Cette liaison est nécéssaire afin de permettre
        // l'utilisation de `this` dans la fonction de rappel.
        this.handleClick = this.handleClick.bind(this);
    }

    handleClick() {
        const { socket } = this.state;
        socket.emit('my SECOND EVENT', { data: 'I\'m connected!' });
    }

    render() {

        const { socket } = this.state;
        socket.on('my response from second event', function () { console.log('test') });

        return (
            <button onClick={this.handleClick}>
                Test
            </button>
        );
    }
}