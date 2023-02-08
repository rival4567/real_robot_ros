#!/usr/bin/env node

const express = require('express');
const path = require('path');
const ejsMate = require('ejs-mate');
const methodOverride = require('method-override');
const ExpressError = require('./utils/ExpressError.js');
const ros = require('./ros/kr-ros.js')

const app = express();
console.log(ros)
app.engine('ejs', ejsMate)
app.set('view engine', 'ejs');
app.set('views', path.join(__dirname, 'views'));
app.use(express.static(path.join(__dirname, 'public')))
app.use(methodOverride('_method'));

app.get('/', (req, res) => {
  res.render('home', { ros: ros });
})

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
