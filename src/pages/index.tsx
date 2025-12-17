import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './styles.module.css';
import Layout from '@theme/Layout';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <p>Comprehensive curriculum covering the latest developments in humanoid robotics research and development</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Start Learning - 5 min ⏱️
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();

  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Description will go into the meta tag in <head />">
      <HomepageHeader />
      <main>
        <section className={styles.features}>
          <div className="container">
            <div className="row">
              <div className="col col--3">
                <h2>Foundations</h2>
                <p>Start with the fundamentals of humanoid robotics, hardware platforms, and development environments.</p>
              </div>
              <div className="col col--3">
                <h2>Simulation & Perception</h2>
                <p>Master simulation environments and perception systems for humanoid robots.</p>
              </div>
              <div className="col col--3">
                <h2>Locomotion & Manipulation</h2>
                <p>Learn locomotion and dexterous manipulation techniques for humanoid robots.</p>
              </div>
              <div className="col col--3">
                <h2>Intelligence & Transfer</h2>
                <p>Explore AI models, sim-to-real transfer, and responsible deployment of humanoid systems.</p>
              </div>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}