# Physical AI Humanoid Robotics Book - Frontend

This directory contains the Docusaurus-based documentation website for the Physical AI Humanoid Robotics Book project.

## Fixes Applied

### 1. Homepage Routing
- Fixed "Page Not Found" error on site root
- Created custom homepage at `src/pages/index.tsx`
- Configured `routeBasePath: 'docs'` in `docusaurus.config.ts` to serve docs at /docs/ and leave / for homepage
- Homepage properly links to documentation at /docs/intro

### 2. Navbar Logo
- Created `static/img/logo.svg` with a simple logo
- Updated navbar configuration in `docusaurus.config.ts` with proper alt text and homepage link
- Logo links to homepage ('/') as expected

### 3. Footer Configuration
- Updated footer title to match site branding
- Verified footer links point to appropriate destinations
- Updated copyright information to match project name

## Features

- **Modern Design**: Clean, professional interface with improved visual hierarchy
- **Enhanced Readability**: Better typography, spacing, and contrast ratios
- **Responsive Design**: Fully responsive layout that works on all device sizes
- **Improved Navigation**: Enhanced sidebar and search functionality
- **Accessibility**: WCAG 2.1 AA compliant with proper contrast ratios and keyboard navigation
- **Performance**: Optimized for fast loading and smooth interactions

## Design Principles

- **Clean Aesthetics**: Modern color palette with blue as the primary color
- **Readable Typography**: Improved font stack with appropriate sizing and line heights
- **Consistent Spacing**: Better spacing and visual rhythm throughout the site
- **Intuitive Navigation**: Enhanced navigation structure for better user orientation
- **Accessibility First**: Designed with accessibility as a core requirement

## Technical Implementation

- Custom CSS variables for consistent theming
- Component swizzling for enhanced navigation elements
- Responsive design using mobile-first approach
- Proper contrast ratios for accessibility compliance
- Modern UI patterns with subtle animations and transitions

## Development

To run the site locally:

```bash
cd frontend_book
npm install
npm run start
```

The site will be available at `http://localhost:3000`

## Customization

The UI can be customized by modifying:

- `src/css/custom.css` - Main styling and theme variables
- `src/theme/` - Custom React components for enhanced functionality
- `docusaurus.config.ts` - Site configuration and theme settings