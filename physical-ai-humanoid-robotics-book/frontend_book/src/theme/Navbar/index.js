import React from 'react';
import Navbar from '@theme-original/Navbar';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import {useLocation} from '@docusaurus/router';

export default function NavbarWrapper(props) {
  const {siteConfig} = useDocusaurusContext();
  const location = useLocation();

  // Add custom navigation elements for enhanced user experience
  const isHomePage = location.pathname === '/';

  return (
    <>
      <Navbar {...props} />
      {/* Enhanced navigation breadcrumbs for better user orientation */}
      {!isHomePage && (
        <div className="navbar__inner container">
          <div className="navbar__items">
            <nav className="navbar__breadcrumb" aria-label="breadcrumbs">
              <ul className="breadcrumbs">
                <li className="breadcrumb__item">
                  <Link to="/" className="breadcrumb__link">
                    {siteConfig.title}
                  </Link>
                </li>
                {/* Breadcrumb items will be dynamically generated based on current page */}
              </ul>
            </nav>
          </div>
        </div>
      )}
    </>
  );
}