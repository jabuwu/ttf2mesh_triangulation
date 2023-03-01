use std::ops::{Deref, DerefMut};

#[derive(Clone, Copy)]
pub(crate) struct LinkedList<T> {
    next: *mut LinkedList<T>,
    prev: *mut LinkedList<T>,
    data: T,
}

impl<T: Default> Default for LinkedList<T> {
    fn default() -> Self {
        Self {
            next: std::ptr::null_mut(),
            prev: std::ptr::null_mut(),
            data: T::default(),
        }
    }
}

impl<T> Deref for LinkedList<T> {
    type Target = T;
    fn deref(&self) -> &Self::Target {
        &self.data
    }
}

impl<T> DerefMut for LinkedList<T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.data
    }
}

impl<T> LinkedList<T> {
    pub fn init(&mut self) {
        self.next = self;
        self.prev = self;
    }

    pub fn first(&self) -> *mut LinkedList<T> {
        self.next
    }

    pub fn next(&self) -> *mut LinkedList<T> {
        self.next
    }

    pub fn set_next(&mut self, next: *mut LinkedList<T>) {
        self.next = next;
    }

    pub fn prev(&self) -> *mut LinkedList<T> {
        self.prev
    }

    pub fn set_prev(&mut self, prev: *mut LinkedList<T>) {
        self.prev = prev;
    }

    pub fn empty(&self) -> bool {
        self.next == self as *const LinkedList<T> as *mut LinkedList<T>
    }

    pub fn insert_after(&mut self, after: *mut LinkedList<T>) {
        unsafe {
            self.prev = after;
            self.next = (*after).next;
            (*(self).prev).next = self;
            (*(self).next).prev = self;
        }
    }

    pub fn insert_last(&mut self, element: *mut LinkedList<T>) {
        unsafe {
            (*element).insert_after(self.prev);
        }
    }

    pub fn detach(&mut self) {
        unsafe {
            (*self.prev).next = self.next;
            (*self.next).prev = self.prev;
        }
    }

    pub fn attach(&mut self, element: *mut LinkedList<T>) {
        unsafe {
            (*element).prev = self;
            (*element).next = self.next;
            (*self.next).prev = element;
            self.next = element;
        }
    }

    pub fn reattach(&mut self, element: *mut LinkedList<T>) {
        unsafe {
            (*element).detach();
            self.attach(element);
        }
    }
}
