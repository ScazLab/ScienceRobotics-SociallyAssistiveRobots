var ros=null;
var answer_msg = null;
var quesnum=0;
var counter=0;
var sub_answer=0;
var ros_pub = null;
var today_game1 = null;
var today_game2 = null;
var today_game3 = null;
var game_id2description = {"0": "Storytelling", "1":"Rocket", "2":"Moon", "3":"Star", "4":"Alien", "5":"House", "6":"Train"};


var question_set =  ["<font size='25px'>Q.1 </font><br><br>What kind of day do you think your child has had today? <br>(1 is a bad day, 3 is typical and 5 is a great day(JS))",
					 "<font size='25px'>Q.2</font><br><br>How easy was it to engage your child with the robot today? <br> (1 is not easy at all, 5 is about average/typical, 10 is super easy)",
					 "<font size='25px'>Q.3</font><br><br>Did you notice their attention was better during one activity more than another? <br><br>If yes, please indicate which activities were most engaging by checking the appropriate checkbox(es)",
					 "<font size='25px'>Q.4</font><br><br>What does your child do or say that lets you know s/he is really enjoying the interaction or activity?  (Check all that apply)",
					 "<font size='25px'>Q.5</font><br><br>How would you rate the difficulty of the activities today for your child <br>(1 is not difficult at all, 3 is a bit challenging, 5 is extremely challenging)",
					 "<font size='25px'>Q.6</font><br><br>How much prompting did you provide your child during the interactions today? <br>(1 is none/extremely little, 3 is a moderate amount, 5 is very frequently)",
					 "<font size='25px'>Q.7</font><br><br>How does this compare with prompting you would typically give in other activities/interactions (1 is much less comparatively, 3 is about the same, 5 is much more comparatively)",
					 "<font size='25px'>Q.8</font><br><br>Have you noticed any changes in your son/daughter's <br><b>eye contact with you?</b> <br>(1 is less, 3 is about the same, 5 is more)",
					 "<font size='25px'>Q.9</font><br><br>Have you noticed any changes in your son/daughter's <br><b>initiation of communication with you?</b> <br>(1 is less, 3 is about the same, 5 is more)",
					 "<font size='25px'>Q.10</font><br><br>Have you noticed any changes in your son/daughter's <br><b>responding to communication bids from you?</b> <br>(1 is less, 3 is about the same, 5 is more)",
					 "<font size='25px'>Q.11</font><br><br>Have you noticed any changes in your son/daughter's <br><b>play initiations with you?</b> <br>(1 is less, 3 is about the same, 5 is more)",
					 "<font size='25px'>Q.12</font><br><br>Have you noticed any changes in your son/daughter's <br><b>eye contact with others?</b> <br>(1 is less, 3 is about the same, 5 is more)",
					 "<font size='25px'>Q.13</font><br><br>Have you noticed any changes in your son/daughter's <br><b>initiation of communication with others?</b> <br>(1 is less, 3 is about the same, 5 is more)",
					 "<font size='25px'>Q.14</font><br><br>Have you noticed any changes in your son/daughter's <br><b>response to communication bids from others?</b> <br>(1 is less, 3 is about the same, 5 is more)",
					 "<font size='25px'>Q.15</font><br><br>Have you noticed any changes in your son/daughter's <br><b>play initiations with others?</b> <br>(1 is less, 3 is about the same, 5 is more)",
					 // "<font size='25px'>Q.16</font><br><br>Are there any other positive/negative behavioral changes you've noticed in your child recently? If yes, please use the fill in the blank to provide details.  Otherwise, please add any other comments in the text box provided.",
					 "<font size='25px'>Q.16</font><br><br>Are there any other positive/negative behavioral changes you've noticed in your child recently? If yes, please use the notepad to provide details.  Otherwise, please write down any other comments on the notepad provided.",
					 ];

ros_setup();

//basic ros connection
function ros_setup() {
  ros = new ROSLIB.Ros({
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

  ros_pub = new ROSLIB.Topic({ //ros publisher
	    ros : ros,
	    name : '/sar/parent_journal',
	    messageType : 'std_msgs/String'
  });

  ros.getParams(function(params) {
    console.log(params);
  });

  today_game1 = new ROSLIB.Param({
    ros : ros,
    name : '/sar/global/_today_game1'
  });
  today_game2 = new ROSLIB.Param({
    ros : ros,
    name : '/sar/global/_today_game2'
  });
  today_game3 = new ROSLIB.Param({
    ros : ros,
    name : '/sar/global/_today_game3'
  });
}

//Dynamically decide the checkbox names.
function q() {
	if(quesnum==2) {
		var _today_game1 = "game 1";
		var _today_game2 = "game 2";
		var _today_game3 = "game 3";

		today_game1.get(function(value) {
    		console.log('game1: ' + value);
    		_today_game1 = value;
  		});
  		today_game2.get(function(value) {
    		console.log('game2: ' + value);
    		_today_game2 = value;
  		});
  		today_game3.get(function(value) {
    		console.log('game3: ' + value);
    		_today_game3 = value;
  		});

		setTimeout(function(){
			document.getElementById("question").innerHTML = question_set[quesnum];
			document.getElementById("responsetype").innerHTML = "<form> \
			<br><br><br><br><input type='radio' name='answer' id='resp1' value='Yes' /><input_text>Yes</input_text><br> \
			<input type='radio' name='answer' id='resp2' value='No' /><input_text>No</input_text><br><br> \
			<input type='checkbox' id='_game1_id'><input_text>" + game_id2description[_today_game1] + "</input_text><br> \
			<input type='checkbox' id='_game2_id'><input_text>" + game_id2description[_today_game2] + "</input_text><br> \
			<input type='checkbox' id='_game3_id'><input_text>" + game_id2description[_today_game3] + "</input_text><br> \
			<br><br><br><input type='button' value='Submit' id='btnsubmit' onclick='p();' style='font-size:20px; width:100px; height:30px'</form>";
		}, 300);

	}
	else if(quesnum==3) {
		document.getElementById("question").innerHTML = question_set[quesnum];
		document.getElementById("responsetype").innerHTML = "<form> \
		<br><br><br><br>\
		<input type='checkbox' name='answer' id='resp11'/><input_text>Smiling</input_text><br> \
		<input type='checkbox' name='answer' id='resp21'/><input_text>Laughing</input_text><br> \
		<input type='checkbox' name='answer' id='resp31'/><input_text>Rocking</input_text><br> \
		<input type='checkbox' name='answer' id='resp41'/><input_text>Flapping</input_text><br> \
		<input type='checkbox' name='answer' id='resp51'/><input_text>Bouncing</input_text><br> \
		<input type='checkbox' name='answer' id='resp61'/><input_text>Raised Voice</input_text><br> \
		<br><br><br><br><input type='button' value='Submit' id='btnsubmit' onclick='p();' style='font-size:20px; width:100px; height:30px'</form>";
	}
	else if(quesnum==15) {
		document.getElementById("question").innerHTML = question_set[quesnum];
		document.getElementById("responsetype").innerHTML = "<button onclick='p()' style='font-size:20px; width:100px; height:30px'>Next</button>";
		// document.getElementById("question").innerHTML = question_set[quesnum];
		// document.getElementById("responsetype").innerHTML = "<form> \
		// <br><br><br><br> \
		// <input type='radio' name='answer' id='resp1' value='Yes' /><input_text>Yes</input_text><br> \
		// <input type='radio' name='answer' id='resp2' value='No' /><input_text>No</input_text><br><br> \
		// <input type='text' name='answer' id='resp3' style='width:400px; height:30px' /> \
		// <br><br><br><br><input type='button' value='Next' id='btnsubmit' onclick='p()' style='font-size:20px; width:100px; height:30px'</form>";
	}
	else if(quesnum<16) {
		document.getElementById("question").innerHTML = question_set[quesnum];
		document.getElementById("responsetype").innerHTML = "<form> \
		<br><br><br><input type='radio' name='answer'  id='resp1' value='1' /><input_text>1</input_text><br> \
		<input type='radio' name='answer' id='resp2' value='2' /><input_text>2</input_text><br> \
		<input type='radio' name='answer' id='resp3' value='3' /><input_text>3</input_text><br> \
		<input type='radio' name='answer' id='resp4' value='4' /><input_text>4</input_text><br> \
		<input type='radio' name='answer' id='resp5' value='5' /><input_text>5</input_text> \
		<br><br><br><br><input type='button' value='Submit' id='btnsubmit' onclick='p();' style='font-size:20px; width:100px; height:30px'</form>";
	}
	else {
		document.getElementById("question").innerHTML = "<font size='25px'></font><br><br>Thanks for completing today's survey!!";

		document.getElementById("responsetype").innerHTML = "<br><br><br><br><button onclick='closing()' style='font-size:20px; width:100px; height:30px'>END</button>";

		// window.close();
	}
	console.log("After q, Quesnum=" + quesnum + ", sub_answer=" + sub_answer);

}
function closing(){
	var done_msg = new ROSLIB.Message({
    	data: 'done'
  	});
	ros_pub.publish(done_msg);
	window.close();
}
function p(){
	if(quesnum==3) {
		if(document.getElementById("resp11").checked) { sub_answer=1; }
		if(document.getElementById("resp21").checked) { sub_answer=2; }
		if(document.getElementById("resp31").checked) { sub_answer=3; }
		if(document.getElementById("resp41").checked) { sub_answer=4; }
		if(document.getElementById("resp51").checked) { sub_answer=5; }
		if(document.getElementById("resp61").checked) { sub_answer=6; }
		quesnum++;
	}
	else if(quesnum==15){
		quesnum++;
	}
	else{
		if(document.getElementById("resp1").checked) { sub_answer=1; document.getElementById("resp1").checked=false; }
		else if(document.getElementById("resp2").checked) { sub_answer=2; document.getElementById("resp2").checked=false; }
		else if(document.getElementById("resp3").checked) { sub_answer=3; document.getElementById("resp3").checked=false; }
		else if(document.getElementById("resp4").checked) { sub_answer=4; document.getElementById("resp4").checked=false; }
		else if(document.getElementById("resp5").checked) { sub_answer=5; document.getElementById("resp5").checked=false; }
		else if(document.getElementById("resp6").checked) { sub_answer=6; document.getElementById("resp6").checked=false; }
		else if(quesnum>15) { sub_answer=7; }

		if(sub_answer==1 || sub_answer==2 || sub_answer==3 || sub_answer==4 || sub_answer==5 || sub_answer==6 || sub_answer==7)
			quesnum++;
		else
			alert("Please enter a value for this question!");
	}

	counter++;
	// var ansvalue=variable;
	if (quesnum == 	3) {
		answer_msg = new ROSLIB.Message({

    				data: 'Question ' + quesnum + ': ' + sub_answer + '. Checks: ' + document.getElementById("_game1_id").checked + ', ' + document.getElementById("_game2_id").checked + ', ' + document.getElementById("_game3_id").checked + '.'
  		});
		console.log(answer_msg);
		ros_pub.publish(answer_msg);

	}
	else if (quesnum==4) {

		answer_msg = new ROSLIB.Message({
    		data: 'Question ' + quesnum + ' checks: ' +document.getElementById("resp11").checked + ', ' +  document.getElementById("resp21").checked + ', ' + document.getElementById("resp31").checked + ', ' + document.getElementById("resp41").checked + ', ' + document.getElementById("resp51").checked + ' ' + document.getElementById("resp61").checked + '.'

  		});
  		console.log(answer_msg);
  		ros_pub.publish(answer_msg);

	}
	else if(quesnum==16){
		// if(sub_answer==1){
		// 	answer_msg = new ROSLIB.Message({
  //   			data: 'Question ' + quesnum + ': ' +sub_answer +'. Comments: "' + document.getElementById("resp3").value + '".'
  // 			});
		// }
		// else {
		// 	answer_msg = new ROSLIB.Message({
  //   			data: 'Question ' + quesnum + ': ' +sub_answer +'. Comments: "' + document.getElementById("resp3").value + '".'
  // 			});
		// }
		// console.log(answer_msg);
		// ros_pub.publish(answer_msg);
		console.log("write down comments on notepad");
	}
	else if (quesnum<17){
		answer_msg = new ROSLIB.Message({
    		data: 'Question ' + quesnum + ': ' +sub_answer + '.'
  		});
		console.log(answer_msg);
		ros_pub.publish(answer_msg);
	}
	q();
}
