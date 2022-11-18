//
// Created by Justin on 18/04/2022.
//

#ifndef FOCX_MAIN_CONTROL_UNIT_WEBPAGE_H
#define FOCX_MAIN_CONTROL_UNIT_WEBPAGE_H

const char* webPage = R"Justin(
<html>
	<head>
		<meta charset="utf-8">
		<title>Meng Meng Robot</title>
		<style type="text/css">
			* {
				-webkit-touch-callout: none;
				-webkit-user-select: none;
				-khtml-user-select: none;
				-moz-user-select: none;
				-ms-user-select: none;
				user-select: none;
			}
			body{
				background-color: #282c35;
			}
			button{
				border-radius: 0.9375rem;
				border: none;
				background-color: #21242d;
				box-shadow: 0 8px 16px 0 rgba(0,0,0,0.2), 0 6px 20px 0 rgba(0,0,0,0.19);
				border: 2px solid #aaffff;
				color: #aaffff;
				font-size:1.625rem;
				width: 8rem;
			}
			button:active {
			  background-color: #282c35;
			  box-shadow: 0 4px 16px 0 rgba(0,0,0,0.2), 0 3px 20px 0 rgba(0,0,0,0.19);
			  transform: translateY(4px);
			}
			#Angle_dec{
				border: 2px solid #ff5500;
				color: #ff5500;
			}
			#buttons{
				height:auto;
				width:auto;
				text-align:center;
				margin: 2.5rem;
			}
			#stream-container,#rocker-container{
				height:auto;
				width:auto;
				text-align:center;
				margin: 0rem;
			}
			#rocker-container{
				margin: 3rem;
			}
			#coordiv{
				border: 2px solid #aaffff;
				border-radius: 0.9375rem;
			}
			#dirBtn {
				border-radius: 0.9375rem;
				border: none;
				background-color: #21242d;
				box-shadow: 0 8px 16px 0 rgba(0,0,0,0.2), 0 6px 20px 0 rgba(0,0,0,0.19);
				border: 3px solid #aaffff;
				color: #aaffff;
				font-size:4.625rem;
				width: 8rem;
				height:8rem;
			}
		</style>
	</head>
	<body>
        <div id="buttons">
			<button id="Angle_inc" onclick="AngleIncButton()">Angle +</button>
			<button id="Angle_dec" onclick="AngleDecButton()">Angle -</button>
		</div>
		<div id="buttonTable" align="center">
			<table id="mainTable" style="width:400px;height:400px;margin:auto;table-layout:fixed;" CELLSPACING=10>
				<tr>
					<td><button id="dirBtn" onmousedown="move(1)" onmouseup="move(0)">&#11017;</button></td>
					<td><button id="dirBtn" onmousedown="move(2)" onmouseup="move(0)">&#8679;</button></td>
					<td><button id="dirBtn" onmousedown="move(3)" onmouseup="move(0)">&#11016;</button></td>
				</tr>

				<tr>
					<td><button id="dirBtn" onmousedown="move(4)" onmouseup="move(0)">&#8678;</button></td>
					<td></td>
					<td><button id="dirBtn" onmousedown="move(5)" onmouseup="move(0)">&#8680;</button></td>
				</tr>

				<tr>
					<td><button id="dirBtn" onmousedown="move(6)" onmouseup="move(0)">&#11019;</button></td>
					<td><button id="dirBtn" onmousedown="move(7)" onmouseup="move(0)">&#8681;</button></td>
					<td><button id="dirBtn" onmousedown="move(8)" onmouseup="move(0)">&#11018;</button></td>
				</tr>
			</table>
		</div>
        <div align="center">
			<p id="heightText" style="color: #aaffff;">Current Height: 155</p>
			<input type="range" min="140" max="230" class="slider" id="heightSlider" value="150" oninput="setHeight(this.value)"/>
		</div>
	</body>
	<script>
		var baseHost = document.location.origin
		document.getElementById("heightText").innerHTML = "Current Height: " + document.getElementById("heightSlider").value;
		function setHeight(h) {
			var heightText = "Current Height: "+ h;
			document.getElementById("heightText").innerHTML = heightText;
			const query = baseHost+"/height?h="+h
				fetch(query)
				  .then(response => {
				    console.log(`request to ${query} finished, status: ${response.status}`)
					})
		}

		function move(a) {
			var x = 0, y = 0;
			var d_x = 45, d_y = -3;
			switch(a) {
				case 0:
					x = 0; y = 0;
				break;
				case 1:
					x = d_x; y = d_y;
				break;
				case 2:
					x = d_x; y = 0;
				break;
				case 3:
					x = d_x; y = -d_y;
				break;
				case 4:
					x = 0; y = d_y;
				break;
				case 5:
					x = 0; y = -d_y;
				break;
				case 6:
					x = -d_x; y = d_y;
				break;
				case 7:
					x = -d_x; y = 0;
				break;
				case 8:
					x = -d_x; y = -d_y;
				break;
			}
			const query = baseHost+"/rocker?x="+x+"&y="+y
			fetch(query)
			  .then(response => {
			    console.log(`request to ${query} finished, status: ${response.status}`)
			  })
		}
        function AngleIncButton(){
            const query = baseHost+"/angle-tune?angle=inc"
            fetch(query)
                .then(response => {
				console.log(`request to ${query} finished, status: ${response.status}`)
            })
        }
		function AngleDecButton(){
			const query = baseHost+"/angle-tune?angle=dec"
			fetch(query)
			  .then(response => {
			    console.log(`request to ${query} finished, status: ${response.status}`)
			})
		}
	</script>
</html>
)Justin";

#endif //FOCX_MAIN_CONTROL_UNIT_WEBPAGE_H
