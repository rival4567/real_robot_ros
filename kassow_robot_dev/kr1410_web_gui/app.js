#!/usr/bin/env node

const express = require('express');
const path = require('path');
const ejsMate = require('ejs-mate');
const methodOverride = require('method-override');
const ExpressError = require('./utils/ExpressError.js');
const autoROS = require('./node_modules/auto-ros/dist/index.js');
const ROS3D = require('ros3d');

console.log(ROS3D)
const ros = new autoROS.default()
ros.connect('ws://10.185.230.104:9090')


const app = express();

// console.log(autoros)
app.engine('ejs', ejsMate)
app.set('view engine', 'ejs');
app.set('views', path.join(__dirname, 'views'));
app.use(express.static(path.join(__dirname, 'public')))
app.use(methodOverride('_method'));

app.get('/', (req, res) => {
  res.render('home');
});

app.all('*', (req, res, next) => {
  next(new ExpressError('Page Not Found', 404))
})

app.use((err, req, res, next) => {
  const { statusCode = 500 } = err;
  if (!err.message) err.message = 'Oh No, Something Went Wrong!'
  res.status(statusCode).render('error', { err })
});

app.listen(3000, () => {
    console.log("Listening to port 3000!")
});
