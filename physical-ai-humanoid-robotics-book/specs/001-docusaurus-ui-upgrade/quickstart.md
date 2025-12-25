# Quickstart: Docusaurus UI Upgrade

## Prerequisites
- Node.js 18.x or higher
- npm 8.x or higher (or yarn 1.22+)
- Git
- Basic knowledge of React and Docusaurus

## Setup and Installation

### 1. Clone the Repository
```bash
git clone <repository-url>
cd physical-ai-humanoid-robotics-book
```

### 2. Navigate to Frontend Book Directory
```bash
cd frontend_book
```

### 3. Install Dependencies
```bash
npm install
# or
yarn install
```

### 4. Run Development Server
```bash
npm run dev
# or
yarn dev
```

The site will be available at `http://localhost:3000`

## Customizing the UI

### 1. Theme Customization
The UI upgrade is implemented through Docusaurus theme customization:

- **Custom CSS**: Located in `src/css/custom.css`
- **Custom Components**: Located in `src/theme/`
- **Theme Configuration**: In `docusaurus.config.js`

### 2. Color Scheme
To customize the color scheme, modify the CSS custom properties in `src/css/custom.css`:

```css
:root {
  --ifm-color-primary: #your-primary-color;
  --ifm-color-primary-dark: #your-darker-shade;
  --ifm-color-primary-darker: #your-darkest-shade;
  --ifm-color-primary-darkest: #your-darkest-shade;
  --ifm-color-primary-light: #your-lighter-shade;
  --ifm-color-primary-lighter: #your-lightest-shade;
  --ifm-color-primary-lightest: #your-lightest-shade;
}
```

### 3. Typography
To customize typography, update the font family and size in `src/css/custom.css`:

```css
:root {
  --ifm-font-family-base: 'Your-Font', system-ui, -apple-system, sans-serif;
  --ifm-font-size-base: 100%; /* Base font size */
  --ifm-line-height-base: 1.7; /* Line height for readability */
}
```

## Navigation Structure

### Sidebar Customization
The sidebar navigation is configured in `sidebars.js`. To modify navigation structure:

1. Open `sidebars.js`
2. Update the sidebar configuration as needed
3. Ensure all existing documentation paths remain unchanged

### Custom Components
Custom navigation components are located in `src/theme/`:
- `Navbar.js` - Custom navigation bar
- `Footer.js` - Custom footer
- `SearchBar.js` - Enhanced search functionality

## Responsive Design

### Breakpoints
The responsive design uses the following breakpoints (defined in `docusaurus.config.js`):

- Small: 0px to 768px (mobile)
- Medium: 769px to 992px (tablet)
- Large: 993px and above (desktop)

### Mobile-First Approach
The design follows a mobile-first approach with progressive enhancement for larger screens. Custom CSS media queries can be added to `src/css/custom.css`:

```css
/* Mobile styles (base) */
.my-component {
  padding: 1rem;
}

/* Tablet styles */
@media (min-width: 769px) {
  .my-component {
    padding: 1.5rem;
  }
}

/* Desktop styles */
@media (min-width: 993px) {
  .my-component {
    padding: 2rem;
  }
}
```

## Accessibility Features

### WCAG 2.1 AA Compliance
The UI upgrade implements the following accessibility features:

1. **Color Contrast**: All text meets minimum contrast ratios
2. **Keyboard Navigation**: All interactive elements are keyboard accessible
3. **Screen Reader Support**: Proper ARIA labels and semantic HTML
4. **Focus Indicators**: Visible focus indicators for keyboard users

### Testing Accessibility
To test accessibility compliance:

1. Run the development server
2. Use accessibility testing tools like axe or Lighthouse
3. Test with screen readers (NVDA, JAWS, VoiceOver)
4. Verify keyboard navigation works properly

## Building for Production

### Build Command
```bash
npm run build
# or
yarn build
```

### Preview Production Build
```bash
npm run serve
# or
yarn serve
```

## Troubleshooting

### Common Issues

1. **Styles not applying**: Clear browser cache and restart development server
2. **Navigation not working**: Verify all sidebar paths match existing documentation structure
3. **Responsive design not working**: Check media query syntax in CSS files
4. **Search not working**: Ensure Algolia or search plugin is properly configured

### Debugging Tips
- Use browser developer tools to inspect elements
- Check browser console for JavaScript errors
- Verify all asset paths are correct
- Ensure all dependencies are properly installed

## Next Steps

1. Customize the color scheme to match your branding
2. Add custom illustrations or diagrams to documentation
3. Implement additional accessibility features as needed
4. Test on various devices and browsers
5. Gather user feedback and iterate on design