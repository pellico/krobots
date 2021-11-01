
// //use std::io::{self, Read, Write, BufRead};
// use std::net::UdpSocket;
// //use std::env;
// //use std::str;
// const buffer_size = 2048;
// fn receiver(buffer:&mut [u8]) -> uint32 {
//     //let socket = UdpSocket::bind("0.0.0.0:2000")?; // for UDP4
//     let socket = UdpSocket::bind("[::]:2000")?;  // for UDP4/6
    

//         // Receives a single datagram message on the socket.
// 	// If `buf` is too small to hold
//         // the message, it will be cut off.
//         let (amt, src) = socket.recv_from(buffer)?;

//         // Redeclare `buf` as slice of the received data
// 	// and send data back to origin.
//         let buf = &mut buf[..amt];
//         socket.send_to(buf, &src)?;
    
// }


// fn main() {
//     let buffer : [u8] = [0;buffer_size];
//     //let socket = UdpSocket::bind("0.0.0.0:2000")?; // for UDP4
//     let socket = UdpSocket::bind("[::]:2000")?;  // for UDP4/6
    

//     // Receives a single datagram message on the socket.
// 	// If `buf` is too small to hold
//         // the message, it will be cut off.
//         let (amt, src) = socket.recv_from(buffer)?;

//         // Redeclare `buf` as slice of the received data
// 	// and send data back to origin.
//         let buf = &mut buf[..amt];
//         socket.send_to(buf, &src)?;

  

// }