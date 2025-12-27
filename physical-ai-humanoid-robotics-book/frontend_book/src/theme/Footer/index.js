import React from 'react';
import Footer from '@theme-original/Footer';

export default function FooterWrapper(props) {
  return (
    <>
      <Footer {...props} />
      {/* Additional footer enhancements for accessibility and user experience */}
      <div className="footer-enhancement">
        <div className="container">
          <div className="footer-bottom">
            <div className="footer-links">
              <a href="#top" className="footer-back-to-top">
                Back to top
              </a>
            </div>
          </div>
        </div>
      </div>
    </>
  );
}