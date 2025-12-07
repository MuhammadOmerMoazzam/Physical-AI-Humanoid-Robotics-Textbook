// docusaurus.config.js
// @ts-check
// Note: This file is for the Docusaurus configuration

const { themes } = require('prism-react-renderer');

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics Textbook',
  tagline: 'The definitive guide to building intelligent humanoid robots',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://muhammadomermoazzam.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<org-name>/<repo-name>'
  baseUrl: '/Physical-AI-Humanoid-Robotics-Textbook/',

  // GitHub pages deployment config.
  organizationName: 'MuhammadOmerMoazzam',
  projectName: 'Physical-AI-Humanoid-Robotics-Textbook',
  trailingSlash: false,
  deploymentBranch: 'gh-pages',

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
            'https://github.com/MuhammadOmerMoazzam/Physical-AI-Humanoid-Robotics-Textbook/tree/main/',
        },
        blog: false, // Disable blog plugin
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],

  plugins: [
    // Plugin for PDF generation
    [
      '@docusaurus/plugin-content-docs',
      {
        id: 'capstone',
        path: 'capstone',
        routeBasePath: 'capstone',
        sidebarPath: require.resolve('./sidebars.js'),
      },
    ],
    // Add other plugins as needed
    [
      '@docusaurus/plugin-client-redirects',
      {
        redirects: [
          {
            to: '/docs/00-setup/',
            from: ['/getting-started'],
          },
        ],
      },
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/docusaurus-social-card.jpg',
      navbar: {
        title: 'Physical AI & Humanoid Robotics',
        logo: {
          alt: 'Robotics Logo',
          src: 'img/logo.svg',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'textbookSidebar',
            position: 'left',
            label: 'Textbook',
          },
          {
            href: 'https://github.com/MuhammadOmerMoazzam/Physical-AI-Humanoid-Robotics-Textbook',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Docs',
            items: [
              {
                label: 'Textbook',
                to: '/docs/00-setup/',
              },
            ],
          },
          {
            title: 'Community',
            items: [
              {
                label: 'Stack Overflow',
                href: 'https://stackoverflow.com/questions/tagged/robotics',
              },
              {
                label: 'Discord',
                href: 'https://discordapp.com/invite/robotics',
              },
              {
                label: 'Twitter',
                href: 'https://twitter.com/robotics',
              },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/MuhammadOmerMoazzam/Physical-AI-Humanoid-Robotics-Textbook',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Textbook. Built with Docusaurus.`,
      },
      prism: {
        theme: themes.github,
        darkTheme: themes.dracula,
        additionalLanguages: ['bash', 'python', 'yaml', 'docker', 'cpp'],
      },
      algolia: {
        // The application ID provided by Algolia
        appId: 'YOUR_APP_ID',
  
        // Public API key: it is safe to commit it
        apiKey: 'YOUR_SEARCH_API_KEY',
  
        indexName: 'your-index-name',
  
        contextualSearch: true,
  
        // Optional: Specify domains where the navigation should occur through window.location instead on history.push. Useful when our Algolia config crawls multiple documentation sites and we want to navigate with window.location.href to them.
        externalUrlRegex: 'external\\.com|domain\\.com',
  
        // Optional: Replace parts of the item URLs from Algolia. Useful when using the same search index for multiple deployments using a different baseUrl. You can use regexp or string in the `from` param. For example: 1. The replacement extends to `searchResult.path` and `searchResult.url` 2. The replacement happens after the prefix is stripped and after the list of `hits.filter` within the Algolia search.
        replaceSearchResultPathname: {
          from: '/docs/', // or as RegExp: /\/docs\//
          to: '/',
        },
  
        // Optional: Algolia search parameters
        searchParameters: {},
  
        // Optional: path for search page that enabled by default (`false` to disable it)
        searchPagePath: 'search',
      },
    }),
};

module.exports = config;