// @ts-check
// `@type` JSDoc annotations allow IDEs and type checkers
// to scan the project for type errors. They're great for
// building large, complex projects that are maintained
// over time.

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics Textbook',
  tagline: 'The Definitive Open Textbook (2025 Edition)',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://physical-ai.org',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/physical-ai-humanoid-robotics-textbook',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'your-org', // Usually your GitHub org/user name.
  projectName: 'physical-ai-humanoid-robotics-textbook', // Usually your repo name.
  trailingSlash: false,

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
        },
        blog: false, // Disable blog for textbook
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/docusaurus-social-card.jpg',
      navbar: {
        title: 'Physical AI Textbook',
        logo: {
          alt: 'Physical AI Logo',
          src: 'img/logo.svg',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Textbook',
          },
          {
            type: 'doc',
            position: 'left',
            label: 'About',
            docId: 'about/intro',
          },
          {
            href: 'https://github.com/your-org/physical-ai-textbook',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Chapters',
            items: [
              {
                label: 'Module 1: Foundations',
                to: '/docs/00-setup',
              },
              {
                label: 'Module 2: Simulation',
                to: '/docs/04-simulation',
              },
              {
                label: 'Module 3: Locomotion',
                to: '/docs/06-locomotion',
              },
              {
                label: 'Module 4: Intelligence',
                to: '/docs/08-vla',
              },
            ],
          },
          {
            title: 'Community',
            items: [
              {
                label: 'Stack Overflow',
                href: 'https://stackoverflow.com/questions/tagged/docusaurus',
              },
              {
                label: 'Discord',
                href: 'https://discordapp.com/invite/docusaurus',
              },
              {
                label: 'Twitter',
                href: 'https://twitter.com/docusaurus',
              },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/your-org/physical-ai-textbook',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Textbook Team. Built with Docusaurus.`,
      },
      prism: {
        theme: require('prism-react-renderer').themes.github,
        darkTheme: require('prism-react-renderer').themes.dracula,
      },
    }),

  plugins: [
    // Plugin for mathematical expressions
    [
      '@docusaurus/plugin-content-docs',
      {
        id: 'appendices',
        path: 'docs/appendices',
        routeBasePath: 'appendices',
        sidebarPath: require.resolve('./sidebars-appendices.js'),
      },
    ],
    // Extra functionality for GitHub Pages
    [
      '@docusaurus/plugin-client-redirects',
      {
        redirects: [
          {
            to: '/docs/00-setup',
            from: ['/docs', '/docs/intro'],
          },
        ],
      },
    ],
    // PDF generation plugin
    [
      '@docusaurus/plugin-pnpm',
      {
        packageManager: 'pnpm',
      }
    ]
  ],
};

module.exports = config;