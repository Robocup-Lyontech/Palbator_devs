import React, { Component } from 'react';
import { connect } from "react-redux"


class App extends Component {

  render() {
    console.log('test')

    return (
      <div className={"App"}>
        <h2>test</h2>
      </div>
    );
  }
}


export default connect(mapStateToProps)(App);
