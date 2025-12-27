import React from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Description will go into the meta tag">
      <main>
        <div className="container">
          <div className="row padding-horiz--md">
            <div className="col col--6 col--offset-3">
              <h1 className="hero__title">{siteConfig.title}</h1>
              <p className="hero__subtitle">{siteConfig.tagline}</p>
              <p>
                Welcome to the Physical AI Humanoid Robotics Book documentation.
                Please navigate to the <a href="/docs/intro">Documentation</a> section to get started.
              </p>
            </div>
          </div>
        </div>
      </main>
    </Layout>
  );
}