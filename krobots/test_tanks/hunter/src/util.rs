use std::{
    pin::Pin,
    task::{Context, Poll},
};




    #[inline]
    pub fn pending_once()->PendingOnce {
        PendingOnce {
            is_ready: false,
          
        }
    }


#[allow(missing_debug_implementations)]
#[doc(hidden)]
pub struct PendingOnce {
    is_ready: bool,
   
}

impl Future for PendingOnce {
    type Output = ();
    fn poll(mut self: Pin<&mut Self>, _: &mut Context<'_>) -> Poll<Self::Output> {
        if self.is_ready {
            Poll::Ready(())
        } else {
            self.is_ready = true;
            Poll::Pending
        }
    }
}
