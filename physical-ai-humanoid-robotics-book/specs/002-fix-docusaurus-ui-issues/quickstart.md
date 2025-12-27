# Quickstart: Fix Docusaurus UI Issues

## Prerequisites
- Node.js 18.x or higher
- npm 8.x or higher (or yarn 1.22+)
- Git
- Basic knowledge of React and Docusaurus

## Setup and Verification

### 1. Clone the Repository (if needed)
```bash
git clone <repository-url>
cd physical-ai-humanoid-robotics-book
cd frontend_book
```

### 2. Install Dependencies
```bash
npm install
# or
yarn install
```

### 3. Run Development Server
```bash
npm run start
# or
yarn start
```

The site will be available at `http://localhost:3000`

## Fixing Homepage Routing

### 1. Check for Index Page
Verify that there's an index page at one of these locations:
- `src/pages/index.js` (Docusaurus page)
- `docs/intro.md` (Docusaurus doc with route: "/")
- Create one if missing

### 2. Update docusaurus.config.js
Check the `presets` section in `docusaurus.config.js` to ensure proper routing:

```javascript
presets: [
  [
    'classic',
    {
      docs: {
        sidebarPath: './sidebars.js',
        // Ensure the intro doc is mapped to root if needed
        // routeBasePath: '/',  // Uncomment if docs should be at root
      },
      blog: false,
      theme: {
        customCss: './src/css/custom.css',
      },
    },
  ],
],
```

### 3. Verify Homepage Route
Make sure the route configuration properly maps the site root to content.

## Fixing Navbar Logo

### 1. Check Logo File
Ensure the logo file exists in the correct location:
- `static/img/logo.svg` (or other image format)

### 2. Update Navbar Configuration
In `docusaurus.config.js`, update the navbar configuration:

```javascript
themeConfig: {
  navbar: {
    title: 'Physical AI Humanoid Robotics',
    logo: {
      alt: 'My Site Logo',
      src: 'img/logo.svg',  // Ensure path is correct relative to static/
    },
    // ... other navbar items
  },
  // ... other theme config
}
```

### 3. Verify Logo Path
- The path should be relative to the `static/` directory
- Common formats: SVG, PNG, JPG
- Ensure the file exists and is accessible

## Fixing Footer Configuration

### 1. Update Footer Configuration
In `docusaurus.config.js`, ensure proper footer configuration:

```javascript
themeConfig: {
  // ... navbar config
  footer: {
    style: 'dark',  // or 'light'
    links: [
      {
        title: 'Docs',
        items: [
          {
            label: 'Tutorial',
            to: '/docs/intro',  // Ensure paths are correct
          },
        ],
      },
      // Add more link sections as needed
    ],
    copyright: `Copyright © ${new Date().getFullYear()} My Project, Inc. Built with Docusaurus.`,
  },
  // ... other theme config
}
```

### 2. Verify Footer Links
- Ensure all link destinations exist
- Check that paths are correct
- Test all footer links work properly

## Testing the Fixes

### 1. Local Testing
1. Run the development server: `npm run start`
2. Navigate to the site root: `http://localhost:3000/`
3. Verify homepage loads without "Page Not Found" error
4. Check that navbar logo displays and links to home
5. Scroll to footer and verify all content and links work

### 2. Production Build Test
```bash
npm run build
npm run serve
```

Test the same functionality on the built site at `http://localhost:3000`

## Troubleshooting

### Common Issues

1. **Logo not displaying**:
   - Verify file exists in `static/img/`
   - Check path in `docusaurus.config.js`
   - Ensure correct file format and permissions

2. **Homepage still showing "Page Not Found"**:
   - Verify index page exists
   - Check route configuration in `docusaurus.config.js`
   - Clear Docusaurus cache: `npm run clear`

3. **Footer links broken**:
   - Verify paths in `docusaurus.config.js`
   - Check if destinations exist
   - Ensure correct URL format

### Debugging Tips
- Use browser developer tools to check for asset loading errors
- Check browser console for JavaScript errors
- Verify all file paths are correct
- Ensure all dependencies are properly installed

## Next Steps

1. Test all fixes thoroughly in development
2. Verify the fixes work across different browsers
3. Test on different screen sizes for responsive design
4. Deploy the fixes to staging/production
5. Verify everything works in the live environment